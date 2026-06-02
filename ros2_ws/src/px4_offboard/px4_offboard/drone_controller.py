#!/usr/bin/env python3
"""
drone_controller.py — ROS2 Offboard Controller cho PX4 SITL
═══════════════════════════════════════════════════════════════
State Machine:
  IDLE → OFFBOARD_ARM → TAKEOFF → MISSION → LAND → DONE

Mission mặc định:
  1. Cất cánh lên 2.5m
  2. Bay tới waypoint (5, 0, 2.5m)
  3. Hover 5 giây
  4. Bay về Home (0, 0, 2.5m)
  5. Hạ cánh

Chạy:
  Terminal 1: ~/px4_sim/launch_sim.sh x500_depth
  Terminal 2: ~/px4_sim/launch_xrce.sh
  Terminal 3: source ~/px4_sim/source_ros2.sh
              ros2 run px4_offboard drone_controller
═══════════════════════════════════════════════════════════════
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected,
)


# ─── Waypoint đơn giản ───────────────────────────────────────────
class WP:
    def __init__(self, x, y, z_ned, yaw_deg=0.0, label=''):
        self.x   = float(x)
        self.y   = float(y)
        self.z   = float(z_ned)       # NED: âm = lên cao
        self.yaw = math.radians(yaw_deg)
        self.label = label or f"({x},{y},{-z_ned}m)"


# ─── Mission mặc định ─────────────────────────────────────────────
DEFAULT_MISSION = [
    WP( 0.0,  0.0, -2.5, yaw_deg=0,   label="Takeoff hold"),
    WP( 5.0,  0.0, -2.5, yaw_deg=0,   label="Waypoint 1 (5m North)"),
    WP( 5.0,  3.0, -2.5, yaw_deg=90,  label="Waypoint 2 (5N, 3E)"),
    WP( 0.0,  0.0, -2.5, yaw_deg=180, label="Return Home"),
]
WP_RADIUS    = 0.4   # m — bán kính chấp nhận waypoint
WP_HOLD_SEC  = 3.0   # giây dừng tại mỗi waypoint


class DroneController(Node):

    CTRL_HZ = 20   # Hz — tần số vòng điều khiển

    def __init__(self):
        super().__init__('drone_controller')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # ── Publishers ────────────────────────────────────────
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',  px4_qos)
        self.pub_cmd      = self.create_publisher(
            VehicleCommand,      '/fmu/in/vehicle_command',       px4_qos)

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_position, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._on_status, px4_qos)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land, px4_qos)

        # ── State ─────────────────────────────────────────────
        self.state   = 'IDLE'
        self.pos     = np.zeros(3)     # [x, y, z] NED
        self.armed   = False
        self.offboard_active = False
        self.landed  = True

        self.warmup_count = 0          # frames trước khi arm
        self.wp_idx   = 0              # waypoint hiện tại
        self.wp_timer = None           # thời điểm đến waypoint

        # Setpoint hiện tại
        self.sp = np.zeros(3)
        self.sp_yaw = 0.0

        self.mission = DEFAULT_MISSION

        self.timer = self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.get_logger().info('🚁 DroneController ready')
        self.get_logger().info(f'   Mission: {len(self.mission)} waypoints')
        for i, wp in enumerate(self.mission):
            self.get_logger().info(f'   WP{i}: {wp.label}')

    # ── Callbacks ─────────────────────────────────────────────────

    def _on_position(self, msg: VehicleLocalPosition):
        self.pos = np.array([msg.x, msg.y, msg.z])

    def _on_status(self, msg: VehicleStatus):
        self.armed           = (msg.arming_state == 2)
        self.offboard_active = (msg.nav_state    == 14)

    def _on_land(self, msg: VehicleLandDetected):
        self.landed = msg.landed

    # ── Main Loop ─────────────────────────────────────────────────

    def _loop(self):
        self._pub_offboard()

        if   self.state == 'IDLE':         self._s_idle()
        elif self.state == 'OFFBOARD_ARM': self._s_offboard_arm()
        elif self.state == 'TAKEOFF':      self._s_takeoff()
        elif self.state == 'MISSION':      self._s_mission()
        elif self.state == 'LAND':         self._s_land()
        elif self.state == 'DONE':         pass

        self._pub_setpoint()

    # ── States ────────────────────────────────────────────────────

    def _s_idle(self):
        """Warm-up 2s rồi switch Offboard + Arm"""
        self.sp = np.array([0.0, 0.0, 0.0])   # giữ mặt đất
        self.warmup_count += 1
        if self.warmup_count >= self.CTRL_HZ * 2:
            self._cmd_set_offboard()
            self._cmd_arm()
            self._go('OFFBOARD_ARM')

    def _s_offboard_arm(self):
        """Chờ drone arm xong + vào Offboard"""
        self.sp = np.array([0.0, 0.0, 0.0])
        if self.armed and self.offboard_active:
            # Bắt đầu takeoff
            first_wp = self.mission[0]
            self.sp     = np.array([first_wp.x, first_wp.y, first_wp.z])
            self.sp_yaw = first_wp.yaw
            self._go('TAKEOFF')

    def _s_takeoff(self):
        """Đợi đạt altitude của WP đầu"""
        target_z = self.mission[0].z
        if abs(self.pos[2] - target_z) < 0.3:
            self.get_logger().info('✅ Takeoff complete — starting mission')
            self.wp_idx   = 1   # WP 0 đã xong (takeoff hold)
            self.wp_timer = None
            self._go('MISSION')

    def _s_mission(self):
        """Fly waypoints theo thứ tự"""
        if self.wp_idx >= len(self.mission):
            self._go('LAND')
            return

        wp = self.mission[self.wp_idx]
        self.sp     = np.array([wp.x, wp.y, wp.z])
        self.sp_yaw = wp.yaw

        # Khoảng cách đến waypoint (bỏ qua z nhỏ)
        dist = np.linalg.norm(self.pos - self.sp)
        if dist < WP_RADIUS:
            if self.wp_timer is None:
                self.wp_timer = self.get_clock().now()
                self.get_logger().info(f'📍 Reached WP{self.wp_idx}: {wp.label}')
            elapsed = (self.get_clock().now() - self.wp_timer).nanoseconds / 1e9
            if elapsed >= WP_HOLD_SEC:
                self.wp_idx  += 1
                self.wp_timer = None
                if self.wp_idx < len(self.mission):
                    nxt = self.mission[self.wp_idx]
                    self.get_logger().info(f'➡  Next WP{self.wp_idx}: {nxt.label}')

    def _s_land(self):
        """Hạ cánh"""
        self.get_logger().info('🛬 Landing...')
        self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if self.landed:
            self._cmd_disarm()
            self._go('DONE')
            self.get_logger().info('✅ Mission complete!')

    # ── PX4 Commands ──────────────────────────────────────────────

    def _pub_offboard(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.timestamp    = self._ts()
        self.pub_offboard.publish(msg)

    def _pub_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position  = [float(v) for v in self.sp]
        msg.yaw       = float(self.sp_yaw)
        msg.timestamp = self._ts()
        self.pub_setpoint.publish(msg)

    def _cmd_set_offboard(self):
        self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)

    def _cmd_arm(self):
        self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)

    def _cmd_disarm(self):
        self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=0.0)

    def _cmd(self, command: int, p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = p1
        msg.param2           = p2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._ts()
        self.pub_cmd.publish(msg)

    def _go(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state

    def _ts(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
