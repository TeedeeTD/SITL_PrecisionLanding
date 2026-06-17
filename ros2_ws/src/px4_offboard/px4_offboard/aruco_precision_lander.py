#!/usr/bin/env python3
"""ArUco precision landing for PX4 x500_gimbal using MAVROS.

This node consumes PoseStamped tracker poses from the ArUco tracker,
performs visual servoing in local ENU, and publishes local ENU position setpoints
and local NED landing targets to MAVROS.
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State, ExtendedState, LandingTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


class ArucoConfig:
    # Flight geometry
    CRUISE_ALT = 15.0
    SEARCH_FRAME = "enu"
    SEARCH_ENU = (3.0, 2.0)
    FINAL_ALT = 0.30
    LANDING_TARGET_Z = 0.0

    # Camera physical offsets relative to drone center (body FLU frame)
    CAMERA_OFFSET_X = 0.1517
    CAMERA_OFFSET_Y = 0.0

    # Timing
    CTRL_HZ = 20
    WARMUP_SEC = 2.0
    GIMBAL_SETTLE_SEC = 4.0
    SEARCH_TIMEOUT = 35.0
    TARGET_TIMEOUT = 1.2
    TARGET_LOSS_GRACE = 1.5
    SEARCH_DIAG_INTERVAL = 2.0
    LAND_WAIT_WARN_INTERVAL = 2.0

    # Descent/control
    DESCENT_RATE = 0.45
    ALIGN_CONFIRM_COUNT = 6
    ALIGN_TIMEOUT = 18.0
    MAX_ALIGN_STEP = 0.45
    MAX_DESCENT_STEP = 0.35
    HIGH_ALT_NOISE_ALT = 8.0
    LOW_ALT_PRECISION_ALT = 5.0
    SERVO_GAIN_HIGH_ALT = 0.35
    SERVO_GAIN_LOW_ALT = 0.75
    POSE_FILTER_WINDOW = 7
    POSE_ALPHA_HIGH_ALT = 0.18
    POSE_ALPHA_LOW_ALT = 0.45

    # Dynamic acceptance gates.
    ALIGN_RADIUS_MIN = 1.20
    ALIGN_RADIUS_MAX = 4.00
    ALIGN_RADIUS_ALT_GAIN = 0.20
    ALIGN_RADIUS_BIAS = 0.15
    DESCENT_RADIUS_MIN = 1.50
    DESCENT_RADIUS_MAX = 5.00
    DESCENT_RADIUS_ALT_GAIN = 0.25
    DESCENT_RADIUS_BIAS = 0.20
    POSE_REJECT_RADIUS_MIN = 12.00
    POSE_REJECT_RADIUS_MAX = 25.00
    POSE_REJECT_RADIUS_ALT_GAIN = 1.50

    # Tracker pose axes.
    CAMERA_X_TO_ENU_EAST_SIGN = 1.0
    CAMERA_Y_TO_ENU_NORTH_SIGN = -1.0
    CAMERA_YAW_FRAME = "body"  # "local" or "body"

    # Force disarm config
    FORCE_DISARM_DELAY = 8.0
    FORCE_DISARM_MAGIC = 21196.0


class ArucoPrecisionLander(Node):
    def __init__(self) -> None:
        super().__init__("aruco_precision_lander")
        self.cfg = ArucoConfig

        self._declare_parameters()
        self._load_parameters()

        # MAVROS state and local position QoS are typically best effort or reliable
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

        # Publishers
        self.pub_setpoint = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.pub_landing_target = self.create_publisher(
            LandingTarget, "/mavros/landing_target/raw", 10
        )
        self.pub_state = self.create_publisher(
            String, "/lander/state", 10
        )

        # Subscribers
        self.create_subscription(
            State, "/mavros/state", self._on_status, state_qos
        )
        self.create_subscription(
            ExtendedState, "/mavros/extended_state", self._on_extended_state, state_qos
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_position, pose_qos
        )
        self.create_subscription(
            PoseStamped, self.pose_topic, self._on_tracker_pose, 10
        )

        # Service Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")

        # Node State
        self.state = "INIT"
        self.pos_enu = np.zeros(3, dtype=float)
        self.q_att = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.sp_enu = np.zeros(3, dtype=float)
        self.sp_yaw = 0.0

        self.armed = False
        self.offboard_active = False
        self.landed = True
        self.landed_state = ExtendedState.LANDED_STATE_UNDEFINED
        self.ground_contact = False
        self.maybe_landed = False
        self.at_rest = False

        self.pose_samples: Deque[np.ndarray] = deque(maxlen=self.pose_filter_window)
        self.raw_rel_enu: Optional[np.ndarray] = None
        self.filtered_rel_enu: Optional[np.ndarray] = None
        self.camera_xy_enu: Optional[np.ndarray] = None
        self.target_enu: Optional[np.ndarray] = None
        self.target_rel_norm = float("inf")
        self.last_pose_time = 0.0
        self.last_pose_frame = ""
        self.last_pose_tvec = np.zeros(3, dtype=float)
        self.pose_count = 0
        self.accepted_pose_count = 0

        self.warmup_count = 0
        self._search_start: Optional[float] = None
        self._align_start: Optional[float] = None
        self._target_lost_start: Optional[float] = None
        self._target_lost_from_state: Optional[str] = None
        self._descent_z_sp = self.cruise_alt
        self._target_counter = 0
        self._centered_count = 0
        self._descent_drift_count = 0
        self._last_search_diag = 0.0
        self._last_land_wait_log = 0.0
        self._land_cmd_sent = False
        self._land_cmd_time: Optional[float] = None
        self._disarm_retry_counter = 0
        self._force_disarm_sent = False
        self._gimbal_control_configured = False

        self.timer = self.create_timer(1.0 / self.ctrl_hz, self._loop)

        self.get_logger().info("ArUco PrecisionLander (MAVROS) ready - state: INIT")
        self.get_logger().info(
            f"Search input ({self.search_frame}): x={self.search_input_xy[0]:.2f}m, "
            f"y={self.search_input_xy[1]:.2f}m"
        )
        self.get_logger().info(
            f"Search ENU east/north=[{self.search_enu_xy[0]:.2f}, {self.search_enu_xy[1]:.2f}]"
        )
        self.get_logger().info(
            "Controller/setpoint frame: ROS ENU; MAVROS owns ENU -> PX4 NED"
        )
        self.get_logger().info(f"Cruise altitude: {self.cruise_alt:.1f}m")
        self.get_logger().info(f"ArUco tracker pose topic: {self.pose_topic}")
        self.get_logger().info(
            f"Vision frame: camera optical -> {self.camera_yaw_frame} ENU, "
            f"signs x/east={self.camera_x_to_east_sign:+.0f}, "
            f"y/north={self.camera_y_to_north_sign:+.0f}"
        )

    def _declare_parameters(self) -> None:
        cfg = self.cfg
        self.declare_parameter("allow_force_disarm", False)
        self.declare_parameter("cruise_alt", cfg.CRUISE_ALT)
        self.declare_parameter("search_x", cfg.SEARCH_ENU[0])
        self.declare_parameter("search_y", cfg.SEARCH_ENU[1])
        self.declare_parameter("search_frame", cfg.SEARCH_FRAME)
        self.declare_parameter("pose_topic", "/aruco_tracker/pose")
        self.declare_parameter("camera_x_to_body_east_sign", cfg.CAMERA_X_TO_ENU_EAST_SIGN)
        self.declare_parameter("camera_y_to_body_north_sign", cfg.CAMERA_Y_TO_ENU_NORTH_SIGN)
        self.declare_parameter("camera_yaw_frame", cfg.CAMERA_YAW_FRAME)
        self.declare_parameter("camera_offset_x", cfg.CAMERA_OFFSET_X)
        self.declare_parameter("camera_offset_y", cfg.CAMERA_OFFSET_Y)

        self.declare_parameter("final_alt", cfg.FINAL_ALT)
        self.declare_parameter("descent_rate", cfg.DESCENT_RATE)
        self.declare_parameter("align_confirm_count", cfg.ALIGN_CONFIRM_COUNT)
        self.declare_parameter("max_align_step", cfg.MAX_ALIGN_STEP)
        self.declare_parameter("max_descent_step", cfg.MAX_DESCENT_STEP)
        self.declare_parameter("pose_filter_window", cfg.POSE_FILTER_WINDOW)
        self.declare_parameter("pose_alpha_high_alt", cfg.POSE_ALPHA_HIGH_ALT)
        self.declare_parameter("pose_alpha_low_alt", cfg.POSE_ALPHA_LOW_ALT)
        self.declare_parameter("servo_gain_high_alt", cfg.SERVO_GAIN_HIGH_ALT)
        self.declare_parameter("servo_gain_low_alt", cfg.SERVO_GAIN_LOW_ALT)
        self.declare_parameter("high_alt_noise_alt", cfg.HIGH_ALT_NOISE_ALT)
        self.declare_parameter("low_alt_precision_alt", cfg.LOW_ALT_PRECISION_ALT)

        self.declare_parameter("align_radius_min", cfg.ALIGN_RADIUS_MIN)
        self.declare_parameter("align_radius_max", cfg.ALIGN_RADIUS_MAX)
        self.declare_parameter("align_radius_alt_gain", cfg.ALIGN_RADIUS_ALT_GAIN)
        self.declare_parameter("align_radius_bias", cfg.ALIGN_RADIUS_BIAS)
        self.declare_parameter("descent_radius_min", cfg.DESCENT_RADIUS_MIN)
        self.declare_parameter("descent_radius_max", cfg.DESCENT_RADIUS_MAX)
        self.declare_parameter("descent_radius_alt_gain", cfg.DESCENT_RADIUS_ALT_GAIN)
        self.declare_parameter("descent_radius_bias", cfg.DESCENT_RADIUS_BIAS)
        self.declare_parameter("pose_reject_radius_min", cfg.POSE_REJECT_RADIUS_MIN)
        self.declare_parameter("pose_reject_radius_max", cfg.POSE_REJECT_RADIUS_MAX)
        self.declare_parameter("pose_reject_radius_alt_gain", cfg.POSE_REJECT_RADIUS_ALT_GAIN)

    def _load_parameters(self) -> None:
        cfg = self.cfg
        self.allow_force_disarm = bool(self.get_parameter("allow_force_disarm").value)
        self.cruise_alt = float(self.get_parameter("cruise_alt").value)
        self.final_alt = float(self.get_parameter("final_alt").value)
        self.descent_rate = float(self.get_parameter("descent_rate").value)
        self.align_confirm_count = int(self.get_parameter("align_confirm_count").value)
        self.max_align_step = float(self.get_parameter("max_align_step").value)
        self.max_descent_step = float(self.get_parameter("max_descent_step").value)
        self.pose_filter_window = max(1, int(self.get_parameter("pose_filter_window").value))
        self.pose_alpha_high_alt = float(self.get_parameter("pose_alpha_high_alt").value)
        self.pose_alpha_low_alt = float(self.get_parameter("pose_alpha_low_alt").value)
        self.servo_gain_high_alt = float(self.get_parameter("servo_gain_high_alt").value)
        self.servo_gain_low_alt = float(self.get_parameter("servo_gain_low_alt").value)
        self.high_alt_noise_alt = float(self.get_parameter("high_alt_noise_alt").value)
        self.low_alt_precision_alt = float(self.get_parameter("low_alt_precision_alt").value)

        self.align_radius_min = float(self.get_parameter("align_radius_min").value)
        self.align_radius_max = float(self.get_parameter("align_radius_max").value)
        self.align_radius_alt_gain = float(self.get_parameter("align_radius_alt_gain").value)
        self.align_radius_bias = float(self.get_parameter("align_radius_bias").value)
        self.descent_radius_min = float(self.get_parameter("descent_radius_min").value)
        self.descent_radius_max = float(self.get_parameter("descent_radius_max").value)
        self.descent_radius_alt_gain = float(self.get_parameter("descent_radius_alt_gain").value)
        self.descent_radius_bias = float(self.get_parameter("descent_radius_bias").value)
        self.pose_reject_radius_min = float(self.get_parameter("pose_reject_radius_min").value)
        self.pose_reject_radius_max = float(self.get_parameter("pose_reject_radius_max").value)
        self.pose_reject_radius_alt_gain = float(self.get_parameter("pose_reject_radius_alt_gain").value)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.camera_x_to_east_sign = float(self.get_parameter("camera_x_to_body_east_sign").value)
        self.camera_y_to_north_sign = float(self.get_parameter("camera_y_to_body_north_sign").value)
        self.camera_yaw_frame = str(self.get_parameter("camera_yaw_frame").value).strip().lower()
        if self.camera_yaw_frame not in ("local", "body"):
            raise ValueError("camera_yaw_frame must be 'local' or 'body'")
        self.camera_offset_x = float(self.get_parameter("camera_offset_x").value)
        self.camera_offset_y = float(self.get_parameter("camera_offset_y").value)

        self.search_frame = str(self.get_parameter("search_frame").value).strip().lower()
        self.search_input_xy = np.array(
            [
                float(self.get_parameter("search_x").value),
                float(self.get_parameter("search_y").value),
            ],
            dtype=float,
        )
        if self.search_frame == "enu":
            self.search_enu_xy = self.search_input_xy.copy()
        elif self.search_frame == "ned":
            self.search_enu_xy = np.array([self.search_input_xy[1], self.search_input_xy[0]], dtype=float)
        else:
            raise ValueError("search_frame must be 'enu' or 'ned'")

        self.ctrl_hz = int(self.cfg.CTRL_HZ)

    def _on_position(self, msg: PoseStamped) -> None:
        self.pos_enu = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=float)

        q = msg.pose.orientation
        self.q_att = np.array([q.w, q.x, q.y, q.z], dtype=float)

    def _on_status(self, msg: State) -> None:
        self.armed = msg.armed
        self.offboard_active = (msg.mode == "OFFBOARD")

    def _on_extended_state(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state
        self.landed = (msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
        self.ground_contact = (msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
        self.maybe_landed = (msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
        self.at_rest = (msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)

    def _on_tracker_pose(self, msg: PoseStamped) -> None:
        self.pose_count += 1
        tvec = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        camera_xy = np.array(
            [
                self.camera_x_to_east_sign * tvec[0],
                self.camera_y_to_north_sign * tvec[1],
            ],
            dtype=float,
        )
        rel_enu = self._camera_xy_to_local_enu(camera_xy)
        reject_radius = self._pose_reject_radius()
        rel_norm = float(np.linalg.norm(rel_enu))

        now = time.time()
        if rel_norm > reject_radius:
            self._log_pose(
                "Tracker pose rejected",
                tvec,
                camera_xy,
                rel_enu,
                extra=f"norm={rel_norm:.2f}m > gate={reject_radius:.2f}m",
            )
            return

        self.camera_xy_enu = camera_xy
        self.raw_rel_enu = rel_enu
        self.pose_samples.append(rel_enu)
        stacked = np.stack(tuple(self.pose_samples), axis=0)
        median_rel_enu = np.median(stacked, axis=0)
        alpha = self._pose_alpha()
        if self.filtered_rel_enu is None:
            self.filtered_rel_enu = median_rel_enu
        else:
            self.filtered_rel_enu = (1.0 - alpha) * self.filtered_rel_enu + alpha * median_rel_enu

        self.target_rel_norm = float(np.linalg.norm(self.filtered_rel_enu))
        target_enu_xy = self._current_enu_xy() + self.filtered_rel_enu
        self.target_enu = np.array(
            [target_enu_xy[0], target_enu_xy[1], self.cfg.LANDING_TARGET_Z],
            dtype=float,
        )

        self.last_pose_time = now
        self.last_pose_frame = msg.header.frame_id or "<empty>"
        self.last_pose_tvec = tvec
        self.accepted_pose_count += 1
        self._log_pose("Tracker pose accepted", tvec, camera_xy, self.filtered_rel_enu)

    def _loop(self) -> None:
        offboard_states = {
            "INIT",
            "TAKEOFF",
            "GIMBAL_DOWN",
            "FLY_TO_SEARCH",
            "SEARCH",
            "HORIZONTAL_APPROACH",
            "DESCEND_OVER_TARGET",
            "TARGET_LOST",
        }

        handler = getattr(self, f"_state_{self.state.lower()}", None)
        if handler is None:
            self.get_logger().error(f"Unknown state: {self.state}")
            return
        handler()

        if self.state in offboard_states:
            self._pub_setpoint()

        try:
            state_msg = String()
            state_msg.data = self.state
            self.pub_state.publish(state_msg)
        except Exception:
            pass

    def _state_init(self) -> None:
        self.sp_enu = np.array([0.0, 0.0, 0.0], dtype=float)
        self.warmup_count += 1
        if self.warmup_count >= self.ctrl_hz * self.cfg.WARMUP_SEC:
            self.warmup_count = 0
            self._transition("TAKEOFF")

    def _state_takeoff(self) -> None:
        if not (self.armed and self.offboard_active):
            self.sp_enu = np.array([0.0, 0.0, self.cruise_alt], dtype=float)
            self.warmup_count += 1
            if self.warmup_count % 10 == 0:
                if not self.offboard_active:
                    self.get_logger().info("Switching to Offboard mode")
                    self._cmd_set_offboard()
                if not self.armed:
                    self.get_logger().info("Arming")
                    self._cmd_arm()
            return

        self.sp_enu = np.array([0.0, 0.0, self.cruise_alt], dtype=float)
        if abs(self.pos_enu[2] - self.cruise_alt) < 0.35:
            self.get_logger().info(
                f"Takeoff complete at {self.pos_enu[2]:.1f}m - pitching gimbal down"
            )
            self.warmup_count = 0
            self._transition("GIMBAL_DOWN")

    def _state_gimbal_down(self) -> None:
        self.sp_enu = np.array([0.0, 0.0, self.cruise_alt], dtype=float)
        if self.warmup_count % 5 == 0:
            self._cmd_gimbal_pitch(-1.5708)
        if self.warmup_count == 0:
            self.get_logger().info("Commanding gimbal pitch to -90 deg")

        self.warmup_count += 1
        if self.warmup_count > self.ctrl_hz * self.cfg.GIMBAL_SETTLE_SEC:
            self.warmup_count = 0
            self.get_logger().info("Gimbal settle complete - flying to search area")
            self._transition("FLY_TO_SEARCH")

    def _state_fly_to_search(self) -> None:
        self.sp_enu = np.array(
            [self.search_enu_xy[0], self.search_enu_xy[1], self.cruise_alt],
            dtype=float,
        )
        dist = float(np.linalg.norm(self.pos_enu[:2] - self.search_enu_xy))
        if dist < 0.60:
            self.get_logger().info("Arrived at search area - starting ArUco search")
            self._reset_visual_filter(clear_pose_time=True)
            self._search_start = time.time()
            self._transition("SEARCH")

    def _state_search(self) -> None:
        self.sp_enu = np.array(
            [self.search_enu_xy[0], self.search_enu_xy[1], self.cruise_alt],
            dtype=float,
        )
        if self._pose_recent():
            self.get_logger().info(
                f"ArUco marker detected: tvec=[{self.last_pose_tvec[0]:.2f}, "
                f"{self.last_pose_tvec[1]:.2f}, {self.last_pose_tvec[2]:.2f}]"
            )
            self._align_start = time.time()
            self._target_counter = 0
            self._centered_count = 0
            self._transition("HORIZONTAL_APPROACH")
            return

        self._log_search_diagnostics()
        if self._search_start and time.time() - self._search_start > self.cfg.SEARCH_TIMEOUT:
            self.get_logger().warn("Search timeout - landing normally at current position")
            req = SetMode.Request()
            req.custom_mode = "AUTO.LAND"
            self.set_mode_client.call_async(req)
            self._transition("LAND")

    def _state_horizontal_approach(self) -> None:
        if not self._pose_recent():
            self._start_target_lost("ArUco target lost during horizontal approach")
            return

        self._send_current_landing_target()
        hold_z = self.cruise_alt
        self.sp_enu = self._visual_setpoint(hold_z, self.max_align_step)

        align_radius = self._align_radius()
        if self.target_rel_norm <= align_radius:
            self._centered_count += 1
        else:
            self._centered_count = 0

        if self._target_counter % self.ctrl_hz == 0:
            self._log_guidance("ArUco visual align", align_radius)
        self._target_counter += 1

        if self._centered_count >= self.align_confirm_count:
            self.get_logger().info(
                f"ArUco target inside dynamic align gate ({align_radius:.2f}m) - descending"
            )
            self._target_counter = 0
            self._centered_count = 0
            self._descent_drift_count = 0
            self._descent_z_sp = float(self.pos_enu[2])
            self._transition("DESCEND_OVER_TARGET")
            return

        if self._align_start and time.time() - self._align_start > self.cfg.ALIGN_TIMEOUT:
            descent_gate = self._descent_radius()
            if self.target_rel_norm <= descent_gate:
                self.get_logger().warn(
                    "High-altitude pose is noisy but inside descent gate; starting guarded descent "
                    f"err={self.target_rel_norm:.2f}m gate={descent_gate:.2f}m"
                )
                self._target_counter = 0
                self._centered_count = 0
                self._descent_z_sp = float(self.pos_enu[2])
                self._transition("DESCEND_OVER_TARGET")
                return
            self.get_logger().warn(
                f"ArUco visual align not centered yet: err={self.target_rel_norm:.2f}m "
                f"gate={align_radius:.2f}m"
            )
            self._align_start = time.time()

    def _state_descend_over_target(self) -> None:
        if not self._pose_recent():
            self._start_target_lost(
                "ArUco target lost during visual descent",
                search_required=False,
            )
            return

        self._send_current_landing_target()

        descent_radius = self._descent_radius()
        descent_allowed = self.target_rel_norm <= descent_radius

        # Low-altitude commitment
        if self.pos_enu[2] < 3.5:
            if not descent_allowed:
                self.get_logger().info(
                    f"Low altitude ({self.pos_enu[2]:.2f}m < 3.5m) - committing to descent despite drift: err={self.target_rel_norm:.2f}m > gate={descent_radius:.2f}m",
                    throttle_duration_sec=1.0,
                )
            descent_allowed = True

        if descent_allowed:
            self._descent_drift_count = 0
            target_final_z = self.final_alt
            self._descent_z_sp = max(
                target_final_z,
                self._descent_z_sp - self.descent_rate / self.ctrl_hz,
            )
        else:
            self._descent_drift_count += 1
            self._descent_z_sp = float(self.pos_enu[2])
            self.get_logger().warn(
                f"ArUco descent paused for realign: err={self.target_rel_norm:.2f}m "
                f"> gate={descent_radius:.2f}m count={self._descent_drift_count}",
                throttle_duration_sec=1.0,
            )
            if self._descent_drift_count >= self.align_confirm_count:
                self._align_start = time.time()
                self._centered_count = 0
                self._target_counter = 0
                self._transition("HORIZONTAL_APPROACH")
                return

        self.sp_enu = self._visual_setpoint(self._descent_z_sp, self.max_descent_step)

        if self._target_counter % self.ctrl_hz == 0:
            phase = "descending" if descent_allowed else "align-hold"
            self._log_guidance(f"ArUco visual descent ({phase})", descent_radius)
        self._target_counter += 1

        if self.pos_enu[2] <= self.final_alt + 0.05:
            self.get_logger().info(
                f"ArUco final altitude reached ({self.final_alt:.2f}m) - switching to PX4 land"
            )
            self._transition("LAND")

    def _state_target_lost(self) -> None:
        if self.pos_enu[2] < 1.5:
            self.get_logger().info(
                f"ArUco target lost at low altitude ({self.pos_enu[2]:.2f}m < 1.5m) - switching to LAND"
            )
            self._transition("LAND")
            return

        if self._target_lost_start is None:
            self._target_lost_start = time.time()

        if self._pose_recent():
            resume = (
                "DESCEND_OVER_TARGET"
                if self._target_lost_from_state == "DESCEND_OVER_TARGET"
                else "HORIZONTAL_APPROACH"
            )
            self.get_logger().info(f"ArUco target reacquired - resuming {resume}")
            self._target_lost_start = None
            self._target_lost_from_state = None
            self._target_counter = 0
            self._centered_count = 0
            if resume == "HORIZONTAL_APPROACH":
                self._align_start = time.time()
            elif resume == "DESCEND_OVER_TARGET":
                self._descent_z_sp = float(self.pos_enu[2])
            self._transition(resume)
            return

        elapsed = time.time() - self._target_lost_start
        if elapsed < self.cfg.TARGET_LOSS_GRACE:
            hold_xy = self.target_enu[:2] if self.target_enu is not None else self.search_enu_xy
            self.sp_enu = np.array([hold_xy[0], hold_xy[1], float(self.pos_enu[2])], dtype=float)
            if self._target_counter % self.ctrl_hz == 0:
                self.get_logger().warn(
                    f"ArUco target temporarily lost - holding last visual setpoint "
                    f"{elapsed:.1f}s/{self.cfg.TARGET_LOSS_GRACE:.1f}s"
                )
            self._target_counter += 1
            return

        self.sp_enu = np.array(
            [self.search_enu_xy[0], self.search_enu_xy[1], self.cruise_alt],
            dtype=float,
        )
        dist = float(np.linalg.norm(self.pos_enu[:2] - self.search_enu_xy))
        alt_err = abs(self.pos_enu[2] - self.cruise_alt)
        if self._target_counter % self.ctrl_hz == 0:
            self.get_logger().warn(
                f"ArUco target lost - returning to search pose: dist={dist:.2f}m "
                f"alt_err={alt_err:.2f}m"
            )
        self._target_counter += 1

        if dist < 0.60 and alt_err < 0.50:
            self._reset_visual_filter(clear_pose_time=True)
            self._search_start = time.time()
            self._target_lost_start = None
            self._target_lost_from_state = None
            self._target_counter = 0
            self._transition("SEARCH")

    def _state_land(self) -> None:
        if self.target_enu is None:
            self.target_enu = np.array(
                [self.search_enu_xy[0], self.search_enu_xy[1], self.cfg.LANDING_TARGET_Z],
                dtype=float,
            )

        if self._pose_recent():
            self._send_current_landing_target()

        if not self._land_cmd_sent:
            req = SetMode.Request()
            req.custom_mode = "AUTO.LAND"
            self.set_mode_client.call_async(req)
            self._land_cmd_sent = True
            self._land_cmd_time = time.time()

        land_elapsed = time.time() - self._land_cmd_time if self._land_cmd_time else 0.0
        near_ground = self.pos_enu[2] < (self.final_alt + 0.08)

        if not self.armed:
            self.get_logger().info("LANDING COMPLETE")
            self._transition("DONE")
            return

        if self.landed:
            if self._disarm_retry_counter % self.ctrl_hz == 0:
                self.get_logger().info("PX4 land detector reports landed - disarming")
                self._cmd_disarm()
            self._disarm_retry_counter += 1
            return

        now = time.time()
        if now - self._last_land_wait_log > self.cfg.LAND_WAIT_WARN_INTERVAL:
            self._last_land_wait_log = now
            self.get_logger().warn(
                "Waiting for PX4 land detector before disarm: "
                f"alt={self.pos_enu[2]:.2f}m, near_ground={near_ground}, "
                f"landed_state={self.landed_state}, elapsed={land_elapsed:.1f}s"
            )

        if (
            self.allow_force_disarm
            and land_elapsed > self.cfg.FORCE_DISARM_DELAY
            and near_ground
            and not self._force_disarm_sent
        ):
            self.get_logger().error(
                "allow_force_disarm=true: PX4 still reports not landed, sending force disarm"
            )
            self._cmd_force_disarm()
            self._force_disarm_sent = True

    def _state_done(self) -> None:
        pass

    def _pose_recent(self) -> bool:
        return (time.time() - self.last_pose_time) < self.cfg.TARGET_TIMEOUT

    def _reset_visual_filter(self, clear_pose_time: bool = False) -> None:
        self.pose_samples.clear()
        self.raw_rel_enu = None
        self.filtered_rel_enu = None
        self.camera_xy_enu = None
        self.target_rel_norm = float("inf")
        self.target_enu = None
        if clear_pose_time:
            self.last_pose_time = 0.0

    def _current_enu_xy(self) -> np.ndarray:
        return self.pos_enu[:2].copy()

    def _altitude(self) -> float:
        return max(0.0, float(self.pos_enu[2]))

    def _altitude_blend(self) -> float:
        span = max(0.1, self.high_alt_noise_alt - self.low_alt_precision_alt)
        return min(1.0, max(0.0, (self._altitude() - self.low_alt_precision_alt) / span))

    def _pose_alpha(self) -> float:
        t = self._altitude_blend()
        return self.pose_alpha_low_alt * (1.0 - t) + self.pose_alpha_high_alt * t

    def _servo_gain(self) -> float:
        t = self._altitude_blend()
        return self.servo_gain_low_alt * (1.0 - t) + self.servo_gain_high_alt * t

    def _align_radius(self) -> float:
        value = self.align_radius_bias + self.align_radius_alt_gain * self._altitude()
        return min(self.align_radius_max, max(self.align_radius_min, value))

    def _descent_radius(self) -> float:
        value = self.descent_radius_bias + self.descent_radius_alt_gain * self._altitude()
        return min(self.descent_radius_max, max(self.descent_radius_min, value))

    def _pose_reject_radius(self) -> float:
        value = self.pose_reject_radius_alt_gain * self._altitude()
        return min(self.pose_reject_radius_max, max(self.pose_reject_radius_min, value))

    def _visual_setpoint(self, target_z: float, max_step: float) -> np.ndarray:
        if self.target_enu is None:
            hold_xy = self.pos_enu[:2]
            return np.array([hold_xy[0], hold_xy[1], target_z], dtype=float)

        current_rel_enu = self.target_enu[:2] - self.pos_enu[:2]
        delta_enu = self._servo_gain() * current_rel_enu
        dist = float(np.linalg.norm(delta_enu))
        if dist > max_step:
            delta_enu *= max_step / dist

        target_enu_xy = self._current_enu_xy() + delta_enu
        return np.array([target_enu_xy[0], target_enu_xy[1], target_z], dtype=float)

    def _send_current_landing_target(self) -> None:
        if self.target_enu is None:
            return

        rel_enu = self.target_enu[:2] - self.pos_enu[:2]
        rel_z = max(float(self.pos_enu[2]), self.final_alt)

        target_x = float(self.target_enu[1]) # North
        target_y = float(self.target_enu[0]) # East
        target_z = float(-self.target_enu[2]) # Down

        rel_x = float(rel_enu[1]) # North
        rel_y = float(rel_enu[0]) # East
        rel_z = float(rel_z) # Down distance

        self._send_landing_target(rel_x, rel_y, rel_z, target_x, target_y, target_z)

    def _send_landing_target(self, rel_x: float, rel_y: float, rel_z: float, target_x: float, target_y: float, target_z: float) -> None:
        try:
            msg = LandingTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.target_num = 0
            msg.frame = LandingTarget.LOCAL_NED
            msg.angle = [0.0, 0.0]
            msg.distance = float(math.sqrt(rel_x * rel_x + rel_y * rel_y + rel_z * rel_z))
            msg.size = [1.0, 1.0]
            msg.pose.position.x = target_x
            msg.pose.position.y = target_y
            msg.pose.position.z = target_z
            msg.pose.orientation.w = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.type = LandingTarget.VISION_FIDUCIAL
            self.pub_landing_target.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"MAVROS LANDING_TARGET send error: {exc}", throttle_duration_sec=2.0)

    def _start_target_lost(self, reason: str, search_required: bool = True) -> None:
        if self.pos_enu[2] < 1.5:
            self.get_logger().info(
                f"ArUco target lost at low altitude ({self.pos_enu[2]:.2f}m < 1.5m) - switching to LAND"
            )
            self._transition("LAND")
            return

        if self.state == "TARGET_LOST":
            if self._target_lost_start is None:
                self._target_lost_start = time.time()
            return

        suffix = "returning to search if not reacquired" if search_required else "holding last visual setpoint"
        self.get_logger().warn(f"{reason} - {suffix}")
        self._target_lost_start = time.time()
        self._target_lost_from_state = self.state
        self._target_counter = 0
        self._transition("TARGET_LOST")

    def _log_search_diagnostics(self) -> None:
        now = time.time()
        if now - self._last_search_diag < self.cfg.SEARCH_DIAG_INTERVAL:
            return
        self._last_search_diag = now
        if self.pose_count == 0:
            self.get_logger().warn(
                f"Waiting for tracker poses on {self.pose_topic}. "
                "Check tracker, camera bridge, and rqt annotated image."
            )
            return
        pose_age = now - self.last_pose_time if self.last_pose_time else float("inf")
        self.get_logger().info(
            f"Tracker poses received={self.pose_count}, accepted={self.accepted_pose_count}, "
            f"last accepted age={pose_age:.1f}s"
        )

    def _log_pose(
        self,
        prefix: str,
        tvec: np.ndarray,
        camera_xy: np.ndarray,
        rel_enu: np.ndarray,
        extra: str = "",
    ) -> None:
        now = time.time()
        if not hasattr(self, "_last_pose_log"):
            self._last_pose_log = 0.0
        if now - self._last_pose_log < 1.0:
            return
        self._last_pose_log = now
        suffix = f" {extra}" if extra else ""
        self.get_logger().info(
            f"{prefix}: frame={self.last_pose_frame or '<pending>'} "
            f"tvec=[{tvec[0]:.2f},{tvec[1]:.2f},{tvec[2]:.2f}] "
            f"camera_xy=[{camera_xy[0]:.2f},{camera_xy[1]:.2f}] "
            f"rel_enu=[{rel_enu[0]:.2f},{rel_enu[1]:.2f}] "
            f"filtered_err={self.target_rel_norm:.2f}m{suffix}"
        )

    def _log_guidance(self, label: str, gate: float) -> None:
        raw = self.raw_rel_enu
        filt = self.filtered_rel_enu
        raw_text = "raw_enu=--" if raw is None else f"raw_enu=[{raw[0]:.2f},{raw[1]:.2f}]"
        filt_text = "filt_enu=--" if filt is None else f"filt_enu=[{filt[0]:.2f},{filt[1]:.2f}]"
        pos_enu = self._current_enu_xy()
        target_enu = (
            self.target_enu[:2]
            if self.target_enu is not None
            else np.array([float("nan"), float("nan")], dtype=float)
        )
        self.get_logger().info(
            f"{label}: alt={self._altitude():.2f}m err={self.target_rel_norm:.2f}m "
            f"gate={gate:.2f}m centered={self._centered_count}/{self.align_confirm_count} "
            f"{raw_text} {filt_text} pos_enu=[{pos_enu[0]:.2f},{pos_enu[1]:.2f}] "
            f"target_enu=[{target_enu[0]:.2f},{target_enu[1]:.2f}] "
            f"sp_enu=[{self.sp_enu[0]:.2f},{self.sp_enu[1]:.2f},{self.sp_enu[2]:.2f}] gain={self._servo_gain():.2f} "
            f"alpha={self._pose_alpha():.2f}"
        )

    def _cmd_gimbal_pitch(self, pitch_rad: float) -> None:
        pitch_deg = math.degrees(pitch_rad)
        if not self._gimbal_control_configured:
            self._cmd(
                1001, # MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
                p1=1.0,
                p2=1.0,
            )
            self._gimbal_control_configured = True

        self._cmd(
            1000, # MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
            p1=pitch_deg,
            p2=0.0,
            p3=math.nan,
            p4=math.nan,
            p5=0.0,
        )
        self._cmd(
            205, # MAV_CMD_DO_MOUNT_CONTROL
            p1=pitch_deg,
            p2=0.0,
            p3=0.0,
            p7=2.0, # MAV_MOUNT_MODE_MAVLINK_TARGETING
        )

    def _pub_setpoint(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self.sp_enu[0])
        msg.pose.position.y = float(self.sp_enu[1])
        msg.pose.position.z = float(self.sp_enu[2])

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(self.sp_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.sp_yaw / 2.0)

        self.pub_setpoint.publish(msg)

    def _cmd_set_offboard(self) -> None:
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.set_mode_client.call_async(req)

    def _cmd_arm(self) -> None:
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

    def _cmd_disarm(self) -> None:
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = False
        self.arming_client.call_async(req)

    def _cmd_force_disarm(self) -> None:
        self._cmd(
            400, # MAV_CMD_COMPONENT_ARM_DISARM
            p1=0.0,
            p2=self.cfg.FORCE_DISARM_MAGIC,
        )

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

    def _transition(self, new_state: str) -> None:
        if new_state == "LAND":
            self._disarm_retry_counter = 0
            self._last_land_wait_log = 0.0
        self.get_logger().info(f"State: {self.state} -> {new_state}")
        self.state = new_state
        try:
            state_msg = String()
            state_msg.data = new_state
            self.pub_state.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish state transition: {e}")

    def _yaw_from_attitude(self) -> float:
        w, x, y, z = [float(v) for v in self.q_att]
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    def _camera_xy_to_local_enu(self, camera_xy: np.ndarray) -> np.ndarray:
        if self.camera_yaw_frame == "local":
            return camera_xy.copy()

        yaw = self._yaw_from_attitude()

        east_body = float(camera_xy[0])
        north_body = float(camera_xy[1])

        x_body = north_body + self.camera_offset_x
        y_body = -east_body + self.camera_offset_y

        c = math.cos(yaw)
        s = math.sin(yaw)

        east_world = x_body * c - y_body * s
        north_world = x_body * s + y_body * c

        return np.array([east_world, north_world], dtype=float)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoPrecisionLander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
