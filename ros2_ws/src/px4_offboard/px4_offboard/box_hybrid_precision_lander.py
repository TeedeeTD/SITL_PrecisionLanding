"""Mission-driven hybrid precision landing skeleton for box integration.

This node follows the real system split:

* PX4/QGroundControl/mission logic flies the UAV to the box region.
* This node watches mission progress and box telemetry.
* Visual guidance starts only after the mission arrival gate and box readiness.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from dib_msgs.msg import LandingTarget6D, BoxTelemetry, BoxState, BoxCmd as BoxCmdMsg
from dib_msgs.srv import BoxCmd as BoxCmdSrv
from mavros_msgs.msg import ExtendedState, LandingTarget, State, WaypointList, WaypointReached
from mavros_msgs.srv import CommandLong, SetMode


BOX_STATE_NAMES = {
    0: "EMPTY",
    1: "IDLE",
    2: "PREPARING_FOR_TAKEOFF",
    3: "MISSION_UPLOADING",
    4: "WAITING_FOR_TAKEOFF",
    5: "WAITING_FOR_RETURN",
    6: "PREPARING_FOR_LANDING",
    7: "WAITING_FOR_LANDING",
    8: "SECURING_DRONE",
    9: "CHARGING",
    10: "MAINTAINING",
    101: "ERROR",
}


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
        self.armed = False
        self.local_pose: Optional[PoseStamped] = None
        self.last_local_pose_time = 0.0
        self.pos_enu = np.zeros(3, dtype=float)
        self.q_att = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.global_position: Optional[NavSatFix] = None
        self.last_global_position_time = 0.0

        self.waypoint_count = 0
        self.last_reached_wp = -1
        self.last_reached_wp_time = 0.0
        self.mission_seen = False
        self.manual_start_requested = False
        self.mission_box_bound = False
        self.mission_trigger_source = ""

        self.box_state = BoxState.EMPTY
        self.box_ready_seen = False
        self.landing_requested = False
        self.last_box_telemetry_time = 0.0
        self.telemetry_box_id = 0
        self.box_latitude = float("nan")
        self.box_longitude = float("nan")
        self.box_yaw = 0.0

        self.target: Optional[LandingTarget6D] = None
        self.last_target_time = 0.0
        self.tracking_count = 0
        self.yaw_aligned_count = 0
        self.search_started_at = 0.0
        self.target_lost_from_state = ""
        self._descent_z_sp = 0.0
        self._descent_drift_count = 0
        self._last_descent_log_time = 0.0

        self.pose_samples: Deque[np.ndarray] = deque(maxlen=7)
        self.filtered_rel_enu: Optional[np.ndarray] = None
        self.target_enu: Optional[np.ndarray] = None
        self.target_rel_norm = float("inf")

        self.gimbal_commanded = False
        self.last_mode_command = ""
        self._last_wait_log_time = 0.0
        self._last_fallback_mode_attempt = 0.0
        self._last_visual_mode_attempt = 0.0
        self._last_published_state: Optional[str] = None
        self._last_state_publish_time = 0.0

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_pub = self.create_publisher(String, "/box_hybrid_landing/drone_state", 10)
        self.lander_state_pub = self.create_publisher(String, "/lander/state", 10)
        self.box_state_pub = self.create_publisher(String, "/box_hybrid_landing/box_state", 10)
        self.comms_pub = self.create_publisher(String, "/box_hybrid_landing/comms", 10)
        self.landing_target_pub = self.create_publisher(LandingTarget, "/mavros/landing_target/raw", 10)
        self.local_setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)

        self.create_subscription(State, "/mavros/state", self._on_mavros_state, state_qos)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_extended_state, state_qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_local_pose, pose_qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._on_global_position, 10)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self._on_waypoint_reached, 10)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self._on_waypoint_list, 10)
        self.create_subscription(String, "/box_hybrid_landing/trigger", self._on_manual_trigger, 10)
        self.create_subscription(LandingTarget6D, self.target_topic, self._on_target, 10)

        # Subscribe to real Box Telemetry instead of String
        self.create_subscription(BoxTelemetry, f"/b{self.box_id}/telemetry", self._on_box_telemetry, 10)

        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.box_cmd_client = self.create_client(BoxCmdSrv, f"/b{self.box_id}/cmd")

        self.timer = self.create_timer(1.0 / self.ctrl_hz, self._tick)

        self.get_logger().info(
            "Box hybrid precision lander ready: "
            f"target_topic={self.target_topic} box_id={self.box_id} drone_id={self.drone_id} "
            f"trigger_mode={self.trigger_mode}"
        )
        self._publish_state(force=True)

    def _declare_parameters(self) -> None:
        self.declare_parameter("target_topic", "/landing/target_camera")
        self.declare_parameter("box_id", 1)
        self.declare_parameter("drone_id", 1)
        self.declare_parameter("ctrl_hz", 20.0)
        self.declare_parameter("marker_size", 0.50)
        self.declare_parameter("tracking_confirm_count", 8)
        self.declare_parameter("yaw_confirm_count", 5)
        self.declare_parameter("target_timeout_sec", 1.2)
        self.declare_parameter("target_loss_grace_sec", 2.5)
        self.declare_parameter("search_timeout_sec", 20.0)
        self.declare_parameter("prelanding_timeout_sec", 10.0)
        self.declare_parameter("box_ready_timeout_sec", 30.0)
        self.declare_parameter("gimbal_settle_sec", 2.0)
        self.declare_parameter("final_alt", 1.0)
        self.declare_parameter("descent_rate", 0.35)
        self.declare_parameter("max_visual_step", 0.35)
        self.declare_parameter("max_descent_step", 0.35)
        self.declare_parameter("xy_gate", 0.30)
        self.declare_parameter("yaw_gate_deg", 5.0)
        self.declare_parameter("align_confirm_count", 6)
        self.declare_parameter("descent_radius_min", 1.50)
        self.declare_parameter("descent_radius_max", 5.00)
        self.declare_parameter("descent_radius_alt_gain", 0.25)
        self.declare_parameter("descent_radius_bias", 0.20)
        self.declare_parameter("enable_yaw_setpoint", False)
        self.declare_parameter("enable_offboard_visual_servo", True)
        self.declare_parameter("trigger_mode", "manual")
        self.declare_parameter("state_heartbeat_sec", 1.0)
        self.declare_parameter("manual_drive_to_box", True)
        self.declare_parameter("manual_drive_alt", 10.0)
        self.declare_parameter("require_in_air_for_mission_trigger", True)
        self.declare_parameter("mission_trigger_min_alt", 1.0)
        self.declare_parameter("fallback_mode", "AUTO.LAND")
        self.declare_parameter("box_telemetry_timeout_sec", 2.0)
        self.declare_parameter("gps_arrival_radius", 2.0)
        self.declare_parameter("enable_sim_local_arrival_gate", True)
        self.declare_parameter("sim_box_target_x", 4.0)
        self.declare_parameter("sim_box_target_y", -3.5)
        self.declare_parameter("sim_arrival_radius", 1.0)

    def _load_parameters(self) -> None:
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.box_id = int(self.get_parameter("box_id").value)
        self.drone_id = int(self.get_parameter("drone_id").value)
        self.ctrl_hz = float(self.get_parameter("ctrl_hz").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.tracking_confirm_count = int(self.get_parameter("tracking_confirm_count").value)
        self.yaw_confirm_count = int(self.get_parameter("yaw_confirm_count").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.target_loss_grace_sec = float(self.get_parameter("target_loss_grace_sec").value)
        self.search_timeout_sec = float(self.get_parameter("search_timeout_sec").value)
        self.prelanding_timeout_sec = float(self.get_parameter("prelanding_timeout_sec").value)
        self.box_ready_timeout_sec = float(self.get_parameter("box_ready_timeout_sec").value)
        self.gimbal_settle_sec = float(self.get_parameter("gimbal_settle_sec").value)
        self.final_alt = float(self.get_parameter("final_alt").value)
        self.descent_rate = float(self.get_parameter("descent_rate").value)
        self.max_visual_step = float(self.get_parameter("max_visual_step").value)
        self.max_descent_step = float(self.get_parameter("max_descent_step").value)
        self.xy_gate = float(self.get_parameter("xy_gate").value)
        self.yaw_gate = math.radians(float(self.get_parameter("yaw_gate_deg").value))
        self.align_confirm_count = int(self.get_parameter("align_confirm_count").value)
        self.descent_radius_min = float(self.get_parameter("descent_radius_min").value)
        self.descent_radius_max = float(self.get_parameter("descent_radius_max").value)
        self.descent_radius_alt_gain = float(self.get_parameter("descent_radius_alt_gain").value)
        self.descent_radius_bias = float(self.get_parameter("descent_radius_bias").value)
        self.enable_yaw_setpoint = bool(self.get_parameter("enable_yaw_setpoint").value)
        self.enable_offboard_visual_servo = bool(self.get_parameter("enable_offboard_visual_servo").value)
        self.trigger_mode = str(self.get_parameter("trigger_mode").value).strip().lower()
        if self.trigger_mode not in ("manual", "mission", "both"):
            self.get_logger().warn(
                f"Invalid trigger_mode='{self.trigger_mode}', falling back to 'manual'"
            )
            self.trigger_mode = "manual"
        self.state_heartbeat_sec = float(self.get_parameter("state_heartbeat_sec").value)
        self.manual_drive_to_box = bool(self.get_parameter("manual_drive_to_box").value)
        self.manual_drive_alt = float(self.get_parameter("manual_drive_alt").value)
        self.require_in_air_for_mission_trigger = bool(
            self.get_parameter("require_in_air_for_mission_trigger").value
        )
        self.mission_trigger_min_alt = float(self.get_parameter("mission_trigger_min_alt").value)
        self.fallback_mode = str(self.get_parameter("fallback_mode").value)
        self.box_telemetry_timeout_sec = float(self.get_parameter("box_telemetry_timeout_sec").value)
        self.gps_arrival_radius = float(self.get_parameter("gps_arrival_radius").value)
        self.enable_sim_local_arrival_gate = bool(self.get_parameter("enable_sim_local_arrival_gate").value)

    def _on_mavros_state(self, msg: State) -> None:
        self.mavros_connected = msg.connected
        self.mode = msg.mode
        self.armed = msg.armed

    def _on_extended_state(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state
        self.landed = msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self.local_pose = msg
        self.last_local_pose_time = self._now()
        self.pos_enu = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        q = msg.pose.orientation
        self.q_att = np.array([q.w, q.x, q.y, q.z], dtype=float)

    def _on_global_position(self, msg: NavSatFix) -> None:
        self.global_position = msg
        self.last_global_position_time = self._now()

    def _on_waypoint_reached(self, msg: WaypointReached) -> None:
        self.last_reached_wp = int(msg.wp_seq)
        self.last_reached_wp_time = self._now()
        self.mission_seen = True

    def _on_waypoint_list(self, msg: WaypointList) -> None:
        self.waypoint_count = len(msg.waypoints)

    def _on_box_telemetry(self, msg: BoxTelemetry) -> None:
        prev_state = self.box_state
        self.box_state = msg.box_state.state
        self.last_box_telemetry_time = self._now()
        self.telemetry_box_id = int(msg.box_info.box_id)
        self.box_latitude = float(msg.box_info.latitude)
        self.box_longitude = float(msg.box_info.longitude)
        self.box_yaw = float(msg.box_info.yaw)
        if self.box_state == BoxState.WAITING_FOR_LANDING:
            self.box_ready_seen = True

        # Publish human-readable box state string
        box_str = BOX_STATE_NAMES.get(self.box_state, f"UNKNOWN({self.box_state})")
        box_msg = String()
        box_msg.data = box_str
        self.box_state_pub.publish(box_msg)

        # Log state transitions to comms topic
        if prev_state != self.box_state:
            prev_str = BOX_STATE_NAMES.get(prev_state, f"UNKNOWN({prev_state})")
            self._publish_comms(f"BOX→DRONE: box_state changed {prev_str} → {box_str}")

    def _on_target(self, msg: LandingTarget6D) -> None:
        now = self._now()
        if msg.tag_id < 0:
            if (now - self.last_target_time) < self.target_timeout_sec:
                return
            self.tracking_count = 0
            return

        self.target = msg
        self.last_target_time = now
        self.tracking_count += 1

        tvec = np.array([msg.x, msg.y, msg.z], dtype=float)
        camera_xy = np.array([1.0 * tvec[0], -1.0 * tvec[1]], dtype=float)
        rel_enu = self._camera_xy_to_local_enu(camera_xy)

        self.pose_samples.append(rel_enu)
        stacked = np.stack(tuple(self.pose_samples), axis=0)
        median_rel_enu = np.median(stacked, axis=0)
        alpha = self._pose_alpha()
        if self.filtered_rel_enu is None:
            self.filtered_rel_enu = median_rel_enu
        else:
            self.filtered_rel_enu = (1.0 - alpha) * self.filtered_rel_enu + alpha * median_rel_enu

        self.target_rel_norm = float(np.linalg.norm(self.filtered_rel_enu))
        target_enu_xy = self.pos_enu[:2] + self.filtered_rel_enu
        self.target_enu = np.array([target_enu_xy[0], target_enu_xy[1], self.final_alt], dtype=float)

    def _tick(self) -> None:
        handler = getattr(self, f"_state_{self.state.lower()}", None)
        if handler is None:
            self.get_logger().error(f"Unknown state {self.state}; entering FALLBACK")
            self._transition("FALLBACK")
            return

        handler()
        self._publish_state()

    def _on_manual_trigger(self, msg: String) -> None:
        """Simulate mission_received + box_id from the box/scheduler side."""
        command = msg.data.strip().lower()
        self.get_logger().info(f"[TRIGGER] Manual trigger received: '{command}' while state={self.state}")
        self._publish_comms(f"USER→DRONE: Trigger received ('{command}'), current_state={self.state}")

        if self.state == "IDLE":
            if command in ("start", "mission", "takeoff", "go", "land"):
                if self.trigger_mode == "mission":
                    self.get_logger().warn("Manual trigger ignored because trigger_mode=mission")
                    self._publish_comms("DRONE: Manual trigger ignored; trigger_mode=mission")
                    return
                self.manual_start_requested = True
                self.get_logger().info("Trigger accepted. Waiting for MAVROS and box telemetry to bind mission target.")
                self._publish_comms("DRONE: Mission received trigger queued; waiting for MAVROS and box telemetry")
            else:
                self.get_logger().warn(f"Ignoring unsupported trigger '{command}' in IDLE")
                self._publish_comms(f"DRONE: Ignored unsupported trigger '{command}' in IDLE")
            return

        self.get_logger().warn(f"Ignoring trigger '{command}' while state={self.state}")

    def _state_idle(self) -> None:
        manual_allowed = self.trigger_mode in ("manual", "both")
        mission_allowed = self.trigger_mode in ("mission", "both")

        manual_triggered = manual_allowed and self.manual_start_requested
        mission_triggered = mission_allowed and self._mission_trigger_ready()

        if not manual_triggered and not mission_triggered:
            return

        if not self.mavros_connected:
            self._log_wait("Landing trigger ready, waiting for /mavros/state connected=true")
            return

        if not self._box_target_from_telemetry_ready():
            self._log_wait("Landing trigger ready, waiting for valid /bX/telemetry box_id and GPS position")
            return

        trigger_source = "manual" if manual_triggered else "mission"
        self.mission_box_bound = True
        self.mission_trigger_source = trigger_source
        self.manual_start_requested = False
        self.get_logger().info(
            "Mission target bound from box telemetry -> DRONE_MISSION: "
            f"source={trigger_source} box_id={self.telemetry_box_id} "
            f"lat={self.box_latitude:.8f} lon={self.box_longitude:.8f}"
        )
        self._publish_comms(
            "BOX→DRONE: Mission target received from telemetry "
            f"source={trigger_source} box_id={self.telemetry_box_id} "
            f"lat={self.box_latitude:.8f} lon={self.box_longitude:.8f}"
        )
        self._transition("DRONE_MISSION")

    def _state_drone_mission(self) -> None:
        """Monitor the mission leg toward the box telemetry target."""
        if not self.mavros_connected:
            self._transition("FALLBACK")
            return

        if not self._box_target_from_telemetry_ready():
            self._log_wait("Mission active, waiting for fresh box telemetry target")
            return

        if self.mission_trigger_source == "manual" and self.manual_drive_to_box:
            self._publish_manual_mission_setpoint()
            if self.mode != "OFFBOARD":
                if self._now() - self._last_visual_mode_attempt >= 1.0:
                    self._last_visual_mode_attempt = self._now()
                    if self._cmd_set_mode("OFFBOARD"):
                        self._publish_comms("DRONE: Manual SITL mission requesting OFFBOARD to fly to box")
                return

        reached_last_wp = self._mission_reached_last_waypoint()
        arrived_by_gps = self._arrived_at_box_by_gps()
        arrived_by_sim_distance = self._arrived_at_box_by_local_distance()

        if reached_last_wp or arrived_by_gps or arrived_by_sim_distance:
            if reached_last_wp:
                reason = "last mission waypoint"
            elif arrived_by_gps:
                reason = "GPS/RTK distance to box telemetry"
            else:
                reason = "SITL local distance gate"
            self.get_logger().info(f"Mission arrival detected by {reason} -> PRELANDING_CHECK")
            self._publish_comms(f"DRONE→BOX: Mission arrival detected by {reason}. Starting PRELANDING_CHECK")
            self._transition("PRELANDING_CHECK")
            return

        if not hasattr(self, "_last_flight_log_time") or (self._now() - self._last_flight_log_time) >= 2.0:
            self._last_flight_log_time = self._now()
            self.get_logger().info(
                "Monitoring PX4 mission to box... "
                f"mode={self.mode} armed={self.armed} "
                f"box_id={self.telemetry_box_id} last_wp={self.last_reached_wp}/{self.waypoint_count - 1} "
                f"{self._box_distance_text()}"
            )

    def _mission_reached_last_waypoint(self) -> bool:
        last_wp = self.waypoint_count - 1 if self.waypoint_count > 0 else -1
        return last_wp >= 0 and self.last_reached_wp >= last_wp

    def _mission_trigger_ready(self) -> bool:
        """Auto landing trigger for the final mission phase.

        This is different from the old auto_start idea: startup alone never
        starts landing. The trigger only becomes true after a real mission
        arrival signal or when the UAV is already inside the box arrival gate.
        """
        if not self.mavros_connected:
            return False
        if not self._box_target_from_telemetry_ready():
            return False
        if self.require_in_air_for_mission_trigger and not self._drone_in_air_for_landing_step():
            return False

        reached_last_wp_now = (
            self._mission_reached_last_waypoint()
            and self.last_reached_wp_time >= self.state_enter_time
        )
        return reached_last_wp_now or self._arrived_at_box_by_gps() or self._arrived_at_box_by_local_distance()

    def _drone_in_air_for_landing_step(self) -> bool:
        if self.landed:
            return False
        if self._altitude() < self.mission_trigger_min_alt:
            return False
        return self.armed or self.landed_state == ExtendedState.LANDED_STATE_IN_AIR

    def _box_target_from_telemetry_ready(self) -> bool:
        if self._now() - self.last_box_telemetry_time > self.box_telemetry_timeout_sec:
            return False
        if self.telemetry_box_id == 0:
            return False
        if self.box_id and self.telemetry_box_id != self.box_id:
            return False
        return self._valid_lat_lon(self.box_latitude, self.box_longitude)

    def _arrived_at_box_by_gps(self) -> bool:
        distance = self._box_distance_gps_m()
        if distance is None:
            return False
        return distance <= self.gps_arrival_radius

    def _arrived_at_box_by_local_distance(self) -> bool:
        if not self.enable_sim_local_arrival_gate:
            return False
        distance = self._box_distance_xy()
        if distance is None:
            return False
        return distance <= float(self.get_parameter("sim_arrival_radius").value)

    def _box_distance_xy(self) -> Optional[float]:
        if not self._local_pose_recent():
            return None
        box_x = float(self.get_parameter("sim_box_target_x").value)
        box_y = float(self.get_parameter("sim_box_target_y").value)
        pos = self.local_pose.pose.position
        return math.hypot(float(pos.x) - box_x, float(pos.y) - box_y)

    def _box_distance_gps_m(self) -> Optional[float]:
        if self.global_position is None or self._now() - self.last_global_position_time > 2.0:
            return None
        if not self._box_target_from_telemetry_ready():
            return None
        lat = float(self.global_position.latitude)
        lon = float(self.global_position.longitude)
        if not self._valid_lat_lon(lat, lon):
            return None
        return self._haversine_m(lat, lon, self.box_latitude, self.box_longitude)

    def _box_distance_text(self) -> str:
        gps_distance = self._box_distance_gps_m()
        local_distance = self._box_distance_xy()
        parts = []
        if gps_distance is not None:
            parts.append(f"gps_box_dist={gps_distance:.2f}m")
        if local_distance is not None:
            parts.append(f"sim_box_dist={local_distance:.2f}m")
        return " ".join(parts) if parts else "box_dist=unknown"

    def _publish_manual_mission_setpoint(self) -> None:
        if self.local_pose is None:
            self._log_wait("Manual SITL mission waiting for /mavros/local_position/pose")
            return

        box_x = float(self.get_parameter("sim_box_target_x").value)
        box_y = float(self.get_parameter("sim_box_target_y").value)
        current_z = float(self.local_pose.pose.position.z)

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = box_x
        sp.pose.position.y = box_y
        sp.pose.position.z = max(current_z, self.manual_drive_alt)
        sp.pose.orientation = self.local_pose.pose.orientation
        self.local_setpoint_pub.publish(sp)

    @staticmethod
    def _valid_lat_lon(latitude: float, longitude: float) -> bool:
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and abs(latitude) > 1e-7
            and abs(longitude) > 1e-7
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )

    @staticmethod
    def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        radius_m = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)
        a = (
            math.sin(d_phi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
        )
        return 2.0 * radius_m * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    def _ensure_visual_control_ready(self) -> bool:
        if not self.enable_offboard_visual_servo:
            return True

        if self.mode == "OFFBOARD":
            return True

        self._publish_current_position_setpoint()
        if self._now() - self._last_visual_mode_attempt >= 1.0:
            self._last_visual_mode_attempt = self._now()
            if self._cmd_set_mode("OFFBOARD"):
                self.get_logger().info("Requesting OFFBOARD for visual servo guidance")
                self._publish_comms("DRONE: Requesting OFFBOARD for visual servo guidance")
        return False

    def _publish_current_position_setpoint(self) -> None:
        if self.local_pose is None:
            return
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position = self.local_pose.pose.position
        sp.pose.orientation = self.local_pose.pose.orientation
        self.local_setpoint_pub.publish(sp)

    def _publish_visual_setpoint(
        self,
        descend: bool,
        use_yaw: bool = False,
        target_z: Optional[float] = None,
        max_step: Optional[float] = None,
    ) -> None:
        if self.local_pose is None or self.target_enu is None:
            return

        pos = self.local_pose.pose.position
        current_rel_enu = self.target_enu[:2] - self.pos_enu[:2]
        delta_enu = self._servo_gain() * current_rel_enu
        dist = float(np.linalg.norm(delta_enu))
        step_limit = self.max_visual_step if max_step is None else max_step
        if dist > step_limit:
            delta_enu *= step_limit / max(dist, 1e-6)

        if target_z is not None:
            z = float(target_z)
        else:
            z = float(pos.z)
        if descend and target_z is None:
            z = max(self.final_alt, z - self.descent_rate / max(self.ctrl_hz, 1.0))

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(pos.x + delta_enu[0])
        sp.pose.position.y = float(pos.y + delta_enu[1])
        sp.pose.position.z = z

        if use_yaw and self.enable_yaw_setpoint:
            target_yaw = self._desired_landing_yaw()
            sp.pose.orientation.x = 0.0
            sp.pose.orientation.y = 0.0
            sp.pose.orientation.z = math.sin(target_yaw / 2.0)
            sp.pose.orientation.w = math.cos(target_yaw / 2.0)
        else:
            sp.pose.orientation = self.local_pose.pose.orientation

        self.local_setpoint_pub.publish(sp)

    def _start_target_lost(self, from_state: str) -> None:
        if self.state == "TARGET_LOST":
            return
        self.target_lost_from_state = from_state
        self._publish_comms(
            f"DRONE: Marker lost during {from_state}; holding position and keeping gimbal down"
        )
        self._transition("TARGET_LOST")

    def _cmd_gimbal_down_throttled(self) -> None:
        if not hasattr(self, "_last_gimbal_cmd_time") or (self._now() - self._last_gimbal_cmd_time) >= 1.0:
            self._last_gimbal_cmd_time = self._now()
            self._cmd_gimbal_down()

    def _state_prelanding_check(self) -> None:
        self._publish_current_position_setpoint()

        # Command gimbal down continuously until confirmed
        self._cmd_gimbal_down_throttled()

        if not self.landing_requested and self._elapsed() >= self.gimbal_settle_sec:
            self._request_landing()

        box_ready = self.box_ready_seen or self.box_state == BoxState.WAITING_FOR_LANDING
        if box_ready and self._elapsed() >= self.gimbal_settle_sec:
            self.get_logger().info("Box is WAITING_FOR_LANDING & gimbal settled -> starting visual SEARCH")
            self._publish_comms("DRONE: Box ready & gimbal pitched down. Transitioning to SEARCH")
            self.search_started_at = self._now()
            self._transition("SEARCH")
            return

        timeout = max(self.prelanding_timeout_sec, self.box_ready_timeout_sec)
        if self._elapsed() > timeout:
            box_str = BOX_STATE_NAMES.get(self.box_state, f"UNKNOWN({self.box_state})")
            self.get_logger().warn(
                "Prelanding check failed or timed out: "
                f"box_ready={box_ready} box_state={box_str} "
                f"landing_requested={self.landing_requested} "
                f"box_cmd_ready={self.box_cmd_client.service_is_ready()}"
            )
            self._transition("FALLBACK")

    def _state_search(self) -> None:
        self._publish_current_position_setpoint()
        self._cmd_gimbal_down_throttled()

        if self._target_recent() and self.tracking_count >= self.tracking_confirm_count:
            self.get_logger().info("ArUco marker detected and tracked! -> HORIZONTAL_APPROACH")
            self._publish_comms("DRONE: ArUco marker acquired! Starting HORIZONTAL_APPROACH")
            self._transition("HORIZONTAL_APPROACH")
            return

        if self.search_started_at and self._now() - self.search_started_at > self.search_timeout_sec:
            self.get_logger().warn("Marker search timeout; transitioning to FALLBACK (GPS/RTK landing)")
            self._publish_comms("DRONE: Marker search timeout; transitioning to FALLBACK (GPS/RTK landing)")
            self._transition("FALLBACK")

    def _state_horizontal_approach(self) -> None:
        self._cmd_gimbal_down_throttled()
        if not self._target_recent():
            self._start_target_lost("HORIZONTAL_APPROACH")
            return

        self._ensure_visual_control_ready()
        self._publish_visual_setpoint(descend=False)
        self._publish_landing_target()
        xy_error = self._target_xy_norm()
        if xy_error <= self.xy_gate:
            self.get_logger().info("Horizontal approach complete -> DESCEND_OVER_TARGET")
            self._descent_z_sp = self._altitude()
            self._descent_drift_count = 0
            self._transition("DESCEND_OVER_TARGET")

    def _state_yaw_align(self) -> None:
        """Optional yaw positioning: keep visual XY closed-loop while yaw settles."""
        self._cmd_gimbal_down_throttled()
        if not self.enable_yaw_setpoint:
            self._transition("DESCEND_OVER_TARGET")
            return

        if not self._target_recent():
            self._start_target_lost("YAW_ALIGN")
            return

        self._ensure_visual_control_ready()
        self._publish_visual_setpoint(descend=False, use_yaw=True)
        self._publish_landing_target()

        xy_error = self._target_xy_norm()
        yaw_error = abs(self._normalize_angle(self._desired_landing_yaw() - self._get_current_yaw()))
        if xy_error <= self.xy_gate and yaw_error <= self.yaw_gate:
            self.yaw_aligned_count += 1
        else:
            self.yaw_aligned_count = 0

        if self.yaw_aligned_count >= self.yaw_confirm_count:
            text = f"Yaw align complete: xy={xy_error:.2f}m yaw_err={math.degrees(yaw_error):.1f}deg -> LAND"
            self.get_logger().info(text)
            self._publish_comms(f"DRONE: {text}")
            self._cmd_gimbal_center()
            self._transition("LAND")

    def _state_descend_over_target(self) -> None:
        self._cmd_gimbal_down_throttled()
        if not self._target_recent():
            self._start_target_lost("DESCEND_OVER_TARGET")
            return

        self._ensure_visual_control_ready()
        self._publish_landing_target()

        descent_radius = self._descent_radius()
        descent_allowed = self.target_rel_norm <= descent_radius
        if self._descent_z_sp <= 0.0:
            self._descent_z_sp = self._altitude()

        if descent_allowed:
            self._descent_drift_count = 0
            self._descent_z_sp = max(
                self.final_alt,
                self._descent_z_sp - self.descent_rate / max(self.ctrl_hz, 1.0),
            )
        else:
            self._descent_drift_count += 1
            self._descent_z_sp = self._altitude()
            if self._now() - self._last_descent_log_time >= 1.0:
                self._last_descent_log_time = self._now()
                self.get_logger().warn(
                    f"Descent paused for realign: err={self.target_rel_norm:.2f}m "
                    f"> gate={descent_radius:.2f}m count={self._descent_drift_count}"
                )
                self._publish_comms(
                    f"DRONE: Descent paused for visual realign err={self.target_rel_norm:.2f}m "
                    f"gate={descent_radius:.2f}m"
                )

        self._publish_visual_setpoint(
            descend=False,
            target_z=self._descent_z_sp,
            max_step=self.max_descent_step,
        )
        if self._altitude() <= self.final_alt:
            if self.enable_yaw_setpoint:
                self.get_logger().info(
                    f"Final altitude reached ({self.final_alt}m)! Holding XY/altitude for final yaw align"
                )
                self.yaw_aligned_count = 0
                self._transition("YAW_ALIGN")
            else:
                self.get_logger().info(f"Final altitude reached ({self.final_alt}m)! Center gimbal & initiating LAND")
                self._cmd_gimbal_center()
                self._transition("LAND")

    def _state_target_lost(self) -> None:
        self._publish_current_position_setpoint()
        self._cmd_gimbal_down_throttled()

        if self._target_recent() and self.tracking_count >= self.tracking_confirm_count:
            resume = self.target_lost_from_state or "HORIZONTAL_APPROACH"
            self.target_lost_from_state = ""
            self._publish_comms(f"DRONE: Marker reacquired, resuming {resume}")
            self._transition(resume)
            return

        if self._elapsed() > self.target_loss_grace_sec:
            self.target_lost_from_state = ""
            self.search_started_at = self._now()
            self._publish_comms("DRONE: Marker still lost, returning to SEARCH instead of immediate fallback")
            self._transition("SEARCH")

    def _state_land(self) -> None:
        self._publish_landing_target()
        if self._cmd_set_mode("AUTO.LAND"):
            self._transition("FLIGHT_IN_PROGRESS")

    def _state_flight_in_progress(self) -> None:
        """Passive non-blocking wait handler while PX4 completes landing."""
        if self._target_recent():
            self._publish_landing_target()

        if not hasattr(self, "_last_land_log_time") or (self._now() - self._last_land_log_time) >= 2.0:
            self._last_land_log_time = self._now()
            self.get_logger().info(
                f"[FLIGHT_IN_PROGRESS] Landing... alt={self._altitude():.2f}m, armed={self.armed}, landed={self.landed}, landed_state={self.landed_state}"
            )

        if self.landed or not self.armed or self.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            self.get_logger().info("Landing complete & UAV disarmed/on ground. Transitioning to DONE")
            self._publish_comms("DRONE: Landing complete on ground. State -> DONE")
            self._transition("DONE")

    def _state_fallback(self) -> None:
        if (
            self.last_mode_command != self.fallback_mode
            or self._now() - self._last_fallback_mode_attempt >= 2.0
        ):
            self._last_fallback_mode_attempt = self._now()
            self._cmd_set_mode(self.fallback_mode)
        if self.landed or not self.armed:
            self._transition("DONE")

    def _state_done(self) -> None:
        """Closed loop reset handler when UAV takes off for the next mission cycle."""
        if not self.landed and self.mavros_connected and (self.armed or self.landed_state == ExtendedState.LANDED_STATE_IN_AIR):
            self.get_logger().info("UAV airborne again! Closing operational loop -> IDLE")
            self._publish_comms("DRONE: UAV airborne again. Operational loop reset -> IDLE")
            self._reset_state()
            self._transition("IDLE")

    def _request_landing(self) -> None:
        if not self.box_cmd_client.service_is_ready():
            self.get_logger().warn(f"Box command service /b{self.box_id}/cmd is not ready")
            return

        req = BoxCmdSrv.Request()
        req.command = BoxCmdMsg.REQUEST_LANDING
        req.agent_id = self.drone_id * 10 + 2
        future = self.box_cmd_client.call_async(req)
        future.add_done_callback(self._on_request_landing_response)
        self.landing_requested = True
        self.get_logger().info(f"Requested landing from Box {self.box_id} with agent_id {req.agent_id}")
        self._publish_comms(
            f"DRONE→BOX: REQUEST_LANDING (cmd=23) box_id={self.box_id} agent_id={req.agent_id}"
        )

    def _on_request_landing_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Box REQUEST_LANDING service call failed: {exc}")
            self._publish_comms(f"BOX→DRONE: REQUEST_LANDING service failed: {exc}")
            return

        if response.success:
            self.get_logger().info("Box accepted REQUEST_LANDING service call")
            self._publish_comms("BOX→DRONE: REQUEST_LANDING accepted")
        else:
            self.get_logger().warn("Box rejected REQUEST_LANDING service call")
            self._publish_comms("BOX→DRONE: REQUEST_LANDING rejected")

    def _publish_comms(self, text: str) -> None:
        """Publish drone↔box communication events for monitoring."""
        msg = String()
        msg.data = text
        self.comms_pub.publish(msg)
        self.get_logger().info(f"[COMMS] {text}")

    def _reset_state(self) -> None:
        self.get_logger().info("Resetting lander node state variables for a new flight")
        self.last_reached_wp = -1
        self.last_reached_wp_time = 0.0
        self.mission_seen = False
        self.manual_start_requested = False
        self.mission_box_bound = False
        self.box_ready_seen = False
        self.landing_requested = False
        self.target = None
        self.tracking_count = 0
        self.yaw_aligned_count = 0
        self.search_started_at = 0.0
        self.target_lost_from_state = ""
        self._descent_z_sp = 0.0
        self._descent_drift_count = 0
        self.pose_samples.clear()
        self.filtered_rel_enu = None
        self.target_enu = None
        self.target_rel_norm = float("inf")
        self.gimbal_commanded = False
        self.last_mode_command = ""

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

        yaw = self._desired_landing_yaw()
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.local_setpoint_pub.publish(msg)

    def _cmd_gimbal_down(self) -> bool:
        if not self.command_client.service_is_ready():
            return False

        if not hasattr(self, "_gimbal_control_configured") or not self._gimbal_control_configured:
            req1 = CommandLong.Request()
            req1.command = 1001  # MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
            req1.param1 = 1.0
            req1.param2 = 191.0  # MAV_COMP_ID_GIMBAL
            self.command_client.call_async(req1)
            self._gimbal_control_configured = True

        req2 = CommandLong.Request()
        req2.command = 1000  # MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        req2.param1 = -90.0
        req2.param2 = 0.0
        req2.param3 = float("nan")
        req2.param4 = float("nan")
        req2.param5 = 0.0
        self.command_client.call_async(req2)

        req3 = CommandLong.Request()
        req3.command = 205  # MAV_CMD_DO_MOUNT_CONTROL
        req3.param1 = -90.0
        req3.param2 = 0.0
        req3.param3 = 0.0
        req3.param7 = 2.0  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        self.command_client.call_async(req3)
        return True

    def _cmd_gimbal_center(self) -> bool:
        if not self.command_client.service_is_ready():
            return False
        req2 = CommandLong.Request()
        req2.command = 1000  # MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        req2.param1 = 0.0    # Reset pitch to 0
        req2.param2 = 0.0
        req2.param3 = float("nan")
        req2.param4 = float("nan")
        req2.param5 = 0.0
        self.command_client.call_async(req2)

        req3 = CommandLong.Request()
        req3.command = 205  # MAV_CMD_DO_MOUNT_CONTROL
        req3.param1 = 0.0
        req3.param2 = 0.0
        req3.param3 = 0.0
        req3.param7 = 2.0
        self.command_client.call_async(req3)
        return True

    def _cmd_set_mode(self, mode: str) -> bool:
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn("SetMode service /mavros/set_mode is not ready!")
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        def _mode_response_cb(fut):
            try:
                res = fut.result()
                if res.mode_sent:
                    self.get_logger().info(f"MAVROS mode change to '{mode}' SUCCESS!")
                else:
                    self.get_logger().warn(f"MAVROS mode change to '{mode}' REJECTED by PX4.")
            except Exception as e:
                self.get_logger().error(f"MAVROS set_mode service call failed: {e}")
        future.add_done_callback(_mode_response_cb)
        self.last_mode_command = mode
        return True

    def _transition(self, new_state: str) -> None:
        if self.state == new_state:
            return
        self.get_logger().info(f"State: {self.state} -> {new_state}")
        self.state = new_state
        self.state_enter_time = self._now()
        self._publish_state(force=True)

    def _publish_state(self, force: bool = False) -> None:
        now = self._now()
        heartbeat_due = (
            self.state_heartbeat_sec > 0.0
            and now - self._last_state_publish_time >= self.state_heartbeat_sec
        )
        if not force and self.state == self._last_published_state and not heartbeat_due:
            return

        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.lander_state_pub.publish(msg)
        self._last_published_state = self.state
        self._last_state_publish_time = now

    def _log_wait(self, text: str, period_sec: float = 2.0) -> None:
        if self._now() - self._last_wait_log_time < period_sec:
            return
        self._last_wait_log_time = self._now()
        self.get_logger().info(text)
        self._publish_comms(f"DRONE: {text}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self.state_enter_time

    def _target_recent(self) -> bool:
        return (
            self.target is not None
            and self._now() - self.last_target_time <= self.target_timeout_sec
        )

    def _local_pose_recent(self) -> bool:
        return self.local_pose is not None and self._now() - self.last_local_pose_time <= 1.0

    def _altitude(self) -> float:
        if self.local_pose is None:
            return float("inf")
        return max(0.0, float(self.local_pose.pose.position.z))

    def _target_xy_norm(self) -> float:
        if math.isfinite(self.target_rel_norm):
            return self.target_rel_norm
        if self.target is None:
            return float("inf")
        return float(np.linalg.norm([self.target.x, self.target.y]))

    def _target_yaw(self) -> float:
        return float(self.target.yaw if self.target is not None else 0.0)

    def _desired_landing_yaw(self) -> float:
        if math.isfinite(self.box_yaw):
            return self._normalize_angle(self.box_yaw)
        return self._normalize_angle(self._get_current_yaw() + self._target_yaw())

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _get_current_yaw(self) -> float:
        if self.local_pose is None:
            return 0.0
        q = self.local_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _yaw_from_attitude(self) -> float:
        w, x, y, z = [float(v) for v in self.q_att]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _camera_xy_to_local_enu(self, camera_xy: np.ndarray) -> np.ndarray:
        yaw = self._yaw_from_attitude()
        east_body = float(camera_xy[0])
        north_body = float(camera_xy[1])
        x_body = north_body + 0.1517
        y_body = -east_body
        c = math.cos(yaw)
        s = math.sin(yaw)
        east_world = x_body * c - y_body * s
        north_world = x_body * s + y_body * c
        return np.array([east_world, north_world], dtype=float)

    def _altitude_blend(self) -> float:
        span = max(0.1, 8.0 - 5.0)
        return min(1.0, max(0.0, (self._altitude() - 5.0) / span))

    def _pose_alpha(self) -> float:
        t = self._altitude_blend()
        return 0.45 * (1.0 - t) + 0.18 * t

    def _servo_gain(self) -> float:
        t = self._altitude_blend()
        return 0.75 * (1.0 - t) + 0.35 * t

    def _descent_radius(self) -> float:
        value = self.descent_radius_bias + self.descent_radius_alt_gain * self._altitude()
        return min(self.descent_radius_max, max(self.descent_radius_min, value))


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
