#!/usr/bin/env python3
"""Lightweight landing target bridge for PX4 precision landing.

This node subscribes to the custom LandingTarget6D messages from the C++ tracker,
converts the target coordinates to the standard MAVROS LandingTarget format (LOCAL_NED),
and publishes them to MAVROS (/mavros/landing_target/raw).
"""

import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dib_msgs.msg import LandingTarget6D
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import LandingTarget, State, ExtendedState
from mavros_msgs.srv import CommandLong


class LandingTargetBridge(Node):
    def __init__(self) -> None:
        super().__init__("landing_target_bridge")

        # Declare parameters
        self.declare_parameter("camera_x_to_body_east_sign", 1.0)
        self.declare_parameter("camera_y_to_body_north_sign", -1.0)
        self.declare_parameter("camera_yaw_frame", "body")  # "local" or "body"
        self.declare_parameter("camera_offset_x", 0.1517)
        self.declare_parameter("camera_offset_y", 0.0)
        self.declare_parameter("marker_size", 0.50)
        self.declare_parameter("target_topic", "/landing/target_camera")

        # Load parameters
        self.camera_x_to_east_sign = self.get_parameter("camera_x_to_body_east_sign").value
        self.camera_y_to_north_sign = self.get_parameter("camera_y_to_body_north_sign").value
        self.camera_yaw_frame = self.get_parameter("camera_yaw_frame").value.strip().lower()
        self.camera_offset_x = self.get_parameter("camera_offset_x").value
        self.camera_offset_y = self.get_parameter("camera_offset_y").value
        self.marker_size = self.get_parameter("marker_size").value
        self.target_topic = self.get_parameter("target_topic").value

        # Node State
        self.pos_enu = np.zeros(3, dtype=float)
        self.q_att = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # w, x, y, z
        self.last_pose_time = 0.0
        self.current_mode = ""
        self.landed_state = 0
        self.is_landing = False

        # Target Filter State
        self.target_samples = deque(maxlen=7)
        self.filtered_target = None
        self.history = deque(maxlen=150)

        # Target Filter Parameters
        self.pose_alpha_high_alt = 0.18
        self.pose_alpha_low_alt = 0.45
        self.high_alt_noise_alt = 8.0
        self.low_alt_precision_alt = 5.0
        self.pose_reject_radius_min = 1.0
        self.pose_reject_radius_max = 5.0
        self.pose_reject_radius_alt_gain = 0.50

        # QoS profiles
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_landing_target = self.create_publisher(
            PoseStamped, "/mavros/landing_target/pose", 10
        )

        # Subscribers
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_position, pose_qos
        )
        self.create_subscription(
            LandingTarget6D, self.target_topic, self._on_tracker_target, 10
        )
        self.create_subscription(
            State, "/mavros/state", self._on_state, state_qos
        )
        self.create_subscription(
            ExtendedState, "/mavros/extended_state", self._on_extended_state, state_qos
        )

        # Gimbal control state
        self._gimbal_control_configured = False
        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")

        # Timer to keep gimbal pitched correctly
        self.create_timer(2.0, self._set_gimbal_pitch)

        self.get_logger().info("Landing Target Bridge initialized.")

    def _on_position(self, msg: PoseStamped) -> None:
        self.pos_enu[0] = msg.pose.position.x
        self.pos_enu[1] = msg.pose.position.y
        self.pos_enu[2] = msg.pose.position.z

        self.q_att[0] = msg.pose.orientation.w
        self.q_att[1] = msg.pose.orientation.x
        self.q_att[2] = msg.pose.orientation.y
        self.q_att[3] = msg.pose.orientation.z

        # Record position and orientation with sim-time stamp for sync
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.history.append((stamp, self.pos_enu.copy(), self.q_att.copy()))

    def _get_historical_state(self, target_time: float) -> tuple[np.ndarray, np.ndarray]:
        if not self.history:
            return self.pos_enu.copy(), self.q_att.copy()
        
        # Find the historical sample closest to target_time
        closest_sample = (self.history[-1][1], self.history[-1][2])
        min_diff = float("inf")
        for t, pos, q in self.history:
            diff = abs(t - target_time)
            if diff < min_diff:
                min_diff = diff
                closest_sample = (pos, q)
        return closest_sample[0].copy(), closest_sample[1].copy()

    def _yaw_from_attitude(self, q: np.ndarray) -> float:
        w, x, y, z = [float(v) for v in q]
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    def _camera_xy_to_local_enu(self, camera_xy: np.ndarray, q_att: np.ndarray) -> np.ndarray:
        if self.camera_yaw_frame == "local":
            return camera_xy.copy()

        # yaw is in ENU (0 is East, positive counter-clockwise)
        yaw = self._yaw_from_attitude(q_att)

        # camera_xy[0] corresponds to drone-right (negative body Y)
        # camera_xy[1] corresponds to drone-forward (positive body X)
        east_body = float(camera_xy[0])
        north_body = float(camera_xy[1])

        # Target position relative to drone center in body FLU (Forward-Left-Up)
        x_body = north_body + self.camera_offset_x
        y_body = -east_body + self.camera_offset_y

        c = math.cos(yaw)
        s = math.sin(yaw)

        # Standard 2D rotation from FLU body to world ENU:
        east_world = x_body * c - y_body * s
        north_world = x_body * s + y_body * c

        return np.array([east_world, north_world], dtype=float)

    def _on_state(self, msg: State) -> None:
        self.current_mode = msg.mode
        self._update_landing_flag()

    def _on_extended_state(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state
        self._update_landing_flag()

    def _update_landing_flag(self) -> None:
        was_landing = self.is_landing
        self.is_landing = (
            (self.current_mode == "AUTO.LAND") or
            (self.landed_state == ExtendedState.LANDED_STATE_LANDING)
        )
        if self.is_landing != was_landing:
            self.get_logger().info(f"Landing status changed: is_landing={self.is_landing} (mode={self.current_mode}, landed_state={self.landed_state})")

    def _on_tracker_target(self, msg: LandingTarget6D) -> None:
        if msg.tag_id < 0 or msg.state == LandingTarget6D.LOST:
            # Clear filter state when tag is lost to avoid jumpy transitions on re-acquisition
            self.target_samples.clear()
            self.filtered_target = None
            return

        tvec = np.array([msg.x, msg.y, msg.z], dtype=float)
        camera_xy = np.array(
            [
                self.camera_x_to_east_sign * tvec[0],
                self.camera_y_to_north_sign * tvec[1],
            ],
            dtype=float,
        )

        # Look up historical vehicle position and attitude at the exact image capture time
        target_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        hist_pos, hist_q = self._get_historical_state(target_time)

        rel_enu = self._camera_xy_to_local_enu(camera_xy, hist_q)
        
        # Outlier rejection check (rejection gate scales with altitude)
        rel_norm = float(np.linalg.norm(rel_enu))
        reject_radius = self._pose_reject_radius()
        if rel_norm > reject_radius:
            self.get_logger().warn(
                f"Tracker target rejected: norm={rel_norm:.2f}m > gate={reject_radius:.2f}m",
                throttle_duration_sec=2.0
            )
            return

        target_enu_xy = hist_pos[:2] + rel_enu

        # Raw absolute coordinates in ENU (East, North, Up)
        raw_target_east = float(target_enu_xy[0])
        raw_target_north = float(target_enu_xy[1])

        # Push to filter queue to smooth out measurement/time-sync noise
        self.target_samples.append(np.array([raw_target_east, raw_target_north], dtype=float))
        stacked = np.stack(tuple(self.target_samples), axis=0)
        median_target = np.median(stacked, axis=0)

        # Exponential moving average filter with dynamic alpha based on altitude
        # High altitude uses smaller alpha for stronger filtering; low altitude uses larger alpha for less lag
        alpha = self._pose_alpha()
        if self.filtered_target is None:
            self.filtered_target = median_target.copy()
        else:
            self.filtered_target = (1.0 - alpha) * self.filtered_target + alpha * median_target

        target_east = float(self.filtered_target[0])
        target_north = float(self.filtered_target[1])
        target_up = 0.0

        try:
            msg_out = PoseStamped()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = "map"

            msg_out.pose.position.x = target_east
            msg_out.pose.position.y = target_north
            msg_out.pose.position.z = target_up
            msg_out.pose.orientation.w = 1.0
            msg_out.pose.orientation.x = 0.0
            msg_out.pose.orientation.y = 0.0
            msg_out.pose.orientation.z = 0.0

            self.pub_landing_target.publish(msg_out)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish MAVROS LandingTarget pose: {exc}", throttle_duration_sec=2.0)

    def _cmd(
        self,
        command: int,
        p1: float = 0.0,
        p2: float = 0.0,
        p3: float = 0.0,
        p4: float = 0.0,
        p5: float = 0.0,
        p6: float = 0.0,
        p7: float = 0.0,
    ) -> None:
        if not self.command_client.service_is_ready():
            return
        req = CommandLong.Request()
        req.command = command
        req.param1 = p1
        req.param2 = p2
        req.param3 = p3
        req.param4 = p4
        req.param5 = p5
        req.param6 = p6
        req.param7 = p7
        self.command_client.call_async(req)

    def _set_gimbal_pitch(self) -> None:
        if not self.command_client.service_is_ready():
            return

        # Configure gimbal manager if not done yet
        if not self._gimbal_control_configured:
            # MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE (1001)
            self._cmd(
                1001,
                p1=1.0,   # Primary control sysid
                p2=191.0, # All components
                p7=0.0,
            )
            self._gimbal_control_configured = True
            self.get_logger().info("Configured gimbal manager control.")

        # Command gimbal to pitch down to -90 degrees only during landing, otherwise 0 degrees (pointing forward)
        pitch_deg = -90.0 if self.is_landing else 0.0
        # MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW (1000)
        self._cmd(
            1000,
            p1=pitch_deg,
            p2=0.0, # Yaw
            p3=float("nan"),
            p4=float("nan"),
            p5=0.0, # Flags
        )
        # MAV_CMD_DO_MOUNT_CONTROL (205)
        self._cmd(
            205,
            p1=pitch_deg,
            p2=0.0,
            p3=0.0,
            p7=2.0, # MAV_MOUNT_MODE_MAVLINK_TARGETING
        )

    def _altitude(self) -> float:
        return max(0.0, float(self.pos_enu[2]))

    def _pose_alpha(self) -> float:
        alt = self._altitude()
        span = max(0.1, self.high_alt_noise_alt - self.low_alt_precision_alt)
        t = min(1.0, max(0.0, (alt - self.low_alt_precision_alt) / span))
        return self.pose_alpha_low_alt * (1.0 - t) + self.pose_alpha_high_alt * t

    def _pose_reject_radius(self) -> float:
        value = self.pose_reject_radius_alt_gain * self._altitude()
        return min(self.pose_reject_radius_max, max(self.pose_reject_radius_min, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandingTargetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Landing Target Bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
