"""Mission-driven hybrid precision landing skeleton for box integration.

This node is intentionally a simulation adapter for the first integration
phase. It keeps MAVROS and tracker dependencies, but uses a simple
``/sim_box/state`` string instead of the full box_manager dib_msgs interface.
The full box telemetry and BoxCmd service can replace that adapter once the
box_manager messages are present in this workspace.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from dib_msgs.msg import LandingTarget6D
from mavros_msgs.msg import ExtendedState, LandingTarget, State, WaypointList, WaypointReached
from mavros_msgs.srv import CommandLong, SetMode


class BoxHybridPrecisionLander(Node):
    def __init__(self) -> None:
        super().__init__("box_hybrid_precision_lander")

        self._declare_parameters()
        self._load_parameters()

        self.state = "IDLE"
        self.state_enter_time = self._now()

        self.mavros_connected = False
        self.landed = False
        self.landed_state = ExtendedState.LANDED_STATE_UNDEFINED
        self.mode = ""
        self.local_pose: Optional[PoseStamped] = None
        self.last_local_pose_time = 0.0

        self.waypoint_count = 0
        self.last_reached_wp = -1
        self.mission_seen = False

        self.box_state = ""
        self.box_ready_seen = False

        self.target: Optional[LandingTarget6D] = None
        self.last_target_time = 0.0
        self.tracking_count = 0
        self.search_started_at = 0.0

        self.gimbal_commanded = False
        self.last_mode_command = ""

        self.state_pub = self.create_publisher(String, "/box_hybrid_landing/state", 10)
        self.landing_target_pub = self.create_publisher(LandingTarget, "/mavros/landing_target/raw", 10)
        self.local_setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)

        self.create_subscription(State, "/mavros/state", self._on_mavros_state, 10)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_extended_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_local_pose, 10)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self._on_waypoint_reached, 10)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self._on_waypoint_list, 10)
        self.create_subscription(LandingTarget6D, self.target_topic, self._on_target, 10)
        self.create_subscription(String, self.sim_box_state_topic, self._on_box_state, 10)

        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.timer = self.create_timer(1.0 / self.ctrl_hz, self._tick)

        self.get_logger().info(
            "Box hybrid precision lander ready: "
            f"target_topic={self.target_topic} sim_box_state_topic={self.sim_box_state_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("auto_start", True)
        self.declare_parameter("target_topic", "/landing/target_camera")
        self.declare_parameter("sim_box_state_topic", "/sim_box/state")
        self.declare_parameter("box_ready_state", "WAITING_FOR_LANDING")
        self.declare_parameter("ctrl_hz", 20.0)
        self.declare_parameter("marker_size", 0.50)
        self.declare_parameter("tracking_confirm_count", 8)
        self.declare_parameter("target_timeout_sec", 0.7)
        self.declare_parameter("search_timeout_sec", 20.0)
        self.declare_parameter("prelanding_timeout_sec", 10.0)
        self.declare_parameter("box_ready_timeout_sec", 30.0)
        self.declare_parameter("gimbal_settle_sec", 2.0)
        self.declare_parameter("final_alt", 1.0)
        self.declare_parameter("xy_gate", 0.30)
        self.declare_parameter("yaw_gate_deg", 8.0)
        self.declare_parameter("enable_yaw_setpoint", False)
        self.declare_parameter("fallback_mode", "AUTO.LAND")

    def _load_parameters(self) -> None:
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.sim_box_state_topic = str(self.get_parameter("sim_box_state_topic").value)
        self.box_ready_state = str(self.get_parameter("box_ready_state").value)
        self.ctrl_hz = float(self.get_parameter("ctrl_hz").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.tracking_confirm_count = int(self.get_parameter("tracking_confirm_count").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.search_timeout_sec = float(self.get_parameter("search_timeout_sec").value)
        self.prelanding_timeout_sec = float(self.get_parameter("prelanding_timeout_sec").value)
        self.box_ready_timeout_sec = float(self.get_parameter("box_ready_timeout_sec").value)
        self.gimbal_settle_sec = float(self.get_parameter("gimbal_settle_sec").value)
        self.final_alt = float(self.get_parameter("final_alt").value)
        self.xy_gate = float(self.get_parameter("xy_gate").value)
        self.yaw_gate = math.radians(float(self.get_parameter("yaw_gate_deg").value))
        self.enable_yaw_setpoint = bool(self.get_parameter("enable_yaw_setpoint").value)
        self.fallback_mode = str(self.get_parameter("fallback_mode").value)

    def _on_mavros_state(self, msg: State) -> None:
        self.mavros_connected = msg.connected
        self.mode = msg.mode

    def _on_extended_state(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state
        self.landed = msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self.local_pose = msg
        self.last_local_pose_time = self._now()

    def _on_waypoint_reached(self, msg: WaypointReached) -> None:
        self.last_reached_wp = int(msg.wp_seq)
        self.mission_seen = True

    def _on_waypoint_list(self, msg: WaypointList) -> None:
        self.waypoint_count = len(msg.waypoints)

    def _on_box_state(self, msg: String) -> None:
        self.box_state = msg.data.strip()
        if self.box_state == self.box_ready_state:
            self.box_ready_seen = True

    def _on_target(self, msg: LandingTarget6D) -> None:
        self.target = msg
        if msg.state == LandingTarget6D.TRACKING:
            self.last_target_time = self._now()
            self.tracking_count += 1
        else:
            self.tracking_count = 0

    def _tick(self) -> None:
        handler = getattr(self, f"_state_{self.state.lower()}", None)
        if handler is None:
            self.get_logger().error(f"Unknown state {self.state}; entering FALLBACK")
            self._transition("FALLBACK")
            return

        handler()
        self._publish_state()

    def _state_idle(self) -> None:
        if not self.mavros_connected:
            return
        if self.auto_start or self.mission_seen or self.waypoint_count > 0:
            self._transition("DRONE_MISSION")

    def _state_drone_mission(self) -> None:
        if not self.mavros_connected:
            self._transition("FALLBACK")
            return

        last_wp = self.waypoint_count - 1 if self.waypoint_count > 0 else -1
        reached_last = last_wp >= 0 and self.last_reached_wp >= last_wp

        if reached_last or (self.auto_start and self._elapsed() > 3.0):
            self._transition("PRELANDING_CHECK")

    def _state_prelanding_check(self) -> None:
        if not self.gimbal_commanded:
            self.gimbal_commanded = self._cmd_gimbal_down()

        pose_ok = self._local_pose_recent()
        tracker_alive = self.target is not None
        gimbal_settled = self._elapsed() >= self.gimbal_settle_sec

        if pose_ok and tracker_alive and gimbal_settled:
            self._transition("WAIT_BOX_READY")
            return

        if self._elapsed() > self.prelanding_timeout_sec:
            self.get_logger().warn(
                "Prelanding check timeout: "
                f"pose_ok={pose_ok} tracker_alive={tracker_alive} gimbal_settled={gimbal_settled}"
            )
            self._transition("FALLBACK")

    def _state_wait_box_ready(self) -> None:
        if self.box_ready_seen or self.box_state == self.box_ready_state:
            self.search_started_at = self._now()
            self._transition("SEARCH")
            return

        if self._elapsed() > self.box_ready_timeout_sec:
            self.get_logger().warn(f"Box did not reach {self.box_ready_state}; entering fallback")
            self._transition("FALLBACK")

    def _state_search(self) -> None:
        if self._target_recent() and self.tracking_count >= self.tracking_confirm_count:
            self._transition("HORIZONTAL_APPROACH")
            return

        if self.search_started_at and self._now() - self.search_started_at > self.search_timeout_sec:
            self.get_logger().warn("Marker search timeout; entering fallback")
            self._transition("FALLBACK")

    def _state_horizontal_approach(self) -> None:
        if not self._target_recent():
            self._transition("TARGET_LOST")
            return

        self._publish_landing_target()
        xy_error = self._target_xy_norm()
        if xy_error <= self.xy_gate:
            self._transition("YAW_ALIGN")

    def _state_yaw_align(self) -> None:
        if not self._target_recent():
            self._transition("TARGET_LOST")
            return

        self._publish_landing_target()
        if self.enable_yaw_setpoint:
            self._publish_yaw_hold_setpoint()

        yaw_error = abs(self._target_yaw())
        if yaw_error <= self.yaw_gate or self._elapsed() > 5.0:
            self._transition("DESCEND_OVER_TARGET")

    def _state_descend_over_target(self) -> None:
        if not self._target_recent():
            self._transition("TARGET_LOST")
            return

        self._publish_landing_target()
        if self._altitude() <= self.final_alt:
            self._transition("LAND")

    def _state_target_lost(self) -> None:
        if self._target_recent() and self.tracking_count >= self.tracking_confirm_count:
            self._transition("HORIZONTAL_APPROACH")
            return

        if self._elapsed() > self.target_timeout_sec:
            self._transition("FALLBACK")

    def _state_land(self) -> None:
        self._publish_landing_target()
        if self._cmd_set_mode("AUTO.LAND"):
            self._transition("FLIGHT_IN_PROGRESS")

    def _state_flight_in_progress(self) -> None:
        if self._target_recent():
            self._publish_landing_target()
        if self.landed:
            self._transition("DONE")

    def _state_fallback(self) -> None:
        if self.last_mode_command != self.fallback_mode:
            self._cmd_set_mode(self.fallback_mode)
        if self.landed:
            self._transition("DONE")

    def _state_done(self) -> None:
        pass

    def _publish_landing_target(self) -> None:
        if self.target is None or self.local_pose is None:
            return

        # LandingTarget6D is in camera optical frame. In this first simulation
        # adapter we use x/y as the relative NED lateral vector after the tracker
        # has already applied the project-specific camera mapping.
        rel_x = float(self.target.y)  # North
        rel_y = float(self.target.x)  # East
        rel_z = max(self._altitude(), self.final_alt)

        pos = self.local_pose.pose.position
        target_x = float(pos.y + rel_x)  # NED North from ENU y
        target_y = float(pos.x + rel_y)  # NED East from ENU x
        target_z = 0.0

        msg = LandingTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.target_num = 0
        msg.frame = LandingTarget.LOCAL_NED
        msg.angle = [0.0, 0.0]
        msg.distance = math.sqrt(rel_x * rel_x + rel_y * rel_y + rel_z * rel_z)
        angular_size = 2.0 * math.atan2(self.marker_size, max(0.01, 2.0 * msg.distance))
        msg.size = [float(angular_size), float(angular_size)]
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z
        msg.pose.orientation.w = 1.0
        msg.type = LandingTarget.VISION_FIDUCIAL
        self.landing_target_pub.publish(msg)

    def _publish_yaw_hold_setpoint(self) -> None:
        if self.local_pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position = self.local_pose.pose.position

        yaw = self._target_yaw()
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.local_setpoint_pub.publish(msg)

    def _cmd_gimbal_down(self) -> bool:
        if not self.command_client.service_is_ready():
            return False

        req = CommandLong.Request()
        req.command = 1000  # MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        req.param1 = -90.0
        req.param2 = 0.0
        req.param5 = 0.0
        self.command_client.call_async(req)
        return True

    def _cmd_set_mode(self, mode: str) -> bool:
        if not self.set_mode_client.service_is_ready():
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.last_mode_command = mode
        return True

    def _transition(self, new_state: str) -> None:
        if self.state == new_state:
            return
        self.get_logger().info(f"State: {self.state} -> {new_state}")
        self.state = new_state
        self.state_enter_time = self._now()

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self.state_enter_time

    def _target_recent(self) -> bool:
        return (
            self.target is not None
            and self.target.state == LandingTarget6D.TRACKING
            and self._now() - self.last_target_time <= self.target_timeout_sec
        )

    def _local_pose_recent(self) -> bool:
        return self.local_pose is not None and self._now() - self.last_local_pose_time <= 1.0

    def _altitude(self) -> float:
        if self.local_pose is None:
            return float("inf")
        return max(0.0, float(self.local_pose.pose.position.z))

    def _target_xy_norm(self) -> float:
        if self.target is None:
            return float("inf")
        return float(np.linalg.norm([self.target.x, self.target.y]))

    def _target_yaw(self) -> float:
        return float(self.target.yaw if self.target is not None else 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxHybridPrecisionLander()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
