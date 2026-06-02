#!/usr/bin/env python3
"""
aruco_precision_lander.py
═══════════════════════════════════════════════════════════════════
UAV ArUco Precision Landing using ROS 2 Offboard position control.

Flow:
  INIT → TAKEOFF → GIMBAL_DOWN → FLY_TO_SEARCH → SEARCH
    → HORIZONTAL_APPROACH → DESCEND_OVER_TARGET → FINAL_APPROACH
    → LAND at 0.1m → DONE

Architecture:
  1. ROS2 node controls offboard takeoff + fly to search area
  2. Gimbal camera pitches down to look at ground
  3. ArUco detection via OpenCV
  4. Publishes MAVLink LANDING_TARGET for PX4-compatible target reporting
  5. Uses PX4-style precision landing phases in Offboard control

Launch:
  Terminal 1: PX4_GZ_WORLD=aruco_landing PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_gimbal
  Terminal 2: MicroXRCEAgent udp4 -p 8888
  Terminal 3: ros2 run ros_gz_bridge parameter_bridge ...  (camera bridge)
  Terminal 4: ros2 run px4_offboard aruco_precision_lander
═══════════════════════════════════════════════════════════════════
"""

import math
import time
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import cv2
import cv2.aruco as aruco
from typing import Optional, Tuple

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude,
    VehicleLandDetected,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ══════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════

class Config:
    # ── Gimbal Camera (from gimbal model.sdf) ─────────────────
    IMG_W   = 1280
    IMG_H   = 720
    HFOV    = 2.0       # radians (~114°)
    # Focal length: fx = (IMG_W/2) / tan(HFOV/2)
    CAM_FX  = (IMG_W / 2) / math.tan(HFOV / 2)  # ≈ 410
    CAM_FY  = CAM_FX
    CAM_CX  = IMG_W / 2.0
    CAM_CY  = IMG_H / 2.0
    CAM_MTX = np.array([
        [CAM_FX, 0, CAM_CX],
        [0, CAM_FY, CAM_CY],
        [0, 0, 1.0]
    ], dtype=np.float32)
    DIST    = np.zeros((4, 1), dtype=np.float32)

    # ── ArUco ─────────────────────────────────────────────────
    # PX4 built-in arucotag uses a specific image — detect using 4x4 dict
    ARUCO_DICT  = aruco.DICT_4X4_50
    MARKER_SIZE = 0.50      # metres, must match Tools/simulation/gz/models/arucotag/model.sdf

    # ── Flight ────────────────────────────────────────────────
    CRUISE_ALT    = 5.0     # m — search altitude
    SEARCH_POS    = (3.0, 2.0)  # approximate marker location (x_north, y_east)

    # ── MAVLink ───────────────────────────────────────────────
    MAVLINK_PORT  = 14540   # PX4 SITL MAVLink port for onboard companion
    MAVLINK_HOST  = '127.0.0.1'

    # ── Gimbal ────────────────────────────────────────────────
    GZ_MODEL_NAME = 'x500_gimbal_0'

    # ── Timing ────────────────────────────────────────────────
    CTRL_HZ       = 20     # Hz
    WARMUP_SEC    = 2.0
    GIMBAL_SETTLE_SEC = 4.0
    SEARCH_TIMEOUT = 30.0  # seconds before giving up search
    TARGET_SEND_HZ = 10    # Hz for LANDING_TARGET messages
    LANDING_TARGET_Z = 0.0
    DESCENT_RATE = 0.45     # m/s
    FINAL_ALT = 0.10        # m above ground before switching to land mode
    FORCE_DISARM_DELAY = 8.0
    FORCE_DISARM_MAGIC = 21196.0
    ALIGN_TIMEOUT = 18.0

    # PX4 precision-landing equivalents. See PX4 PLD_HACC_RAD, PLD_BTOUT,
    # PLD_FAPPR_ALT, PLD_MAX_SRCH, and PLD_SRCH_ALT.
    PLD_HACC_RAD = 0.25
    PLD_BTOUT = 2.0
    PLD_FAPPR_ALT = 0.10
    PLD_MAX_SRCH = 3
    PLD_SRCH_ALT = 5.0
    DESCENT_HACC_RAD = 0.35
    CENTER_CONFIRM_COUNT = 8
    DESCENT_LOSS_GRACE = 8.0
    LOST_TARGET_DESCENT_RATE = 0.25

    MARKER_LOST_TIMEOUT = 1.5
    VISION_TARGET_ALPHA = 0.25
    MAX_TARGET_OFFSET_FROM_SEARCH = 1.5
    MAX_VISUAL_CORRECTION = 2.5
    MAX_APPROACH_STEP = 0.65
    MAX_DESCENT_STEP = 0.35


# ══════════════════════════════════════════════════════════════════
#  MAIN NODE
# ══════════════════════════════════════════════════════════════════

class ArucoPrecisionLander(Node):

    def __init__(self):
        super().__init__('aruco_precision_lander')
        # Enable simulation time programmatically so timestamps match PX4 SITL simulation time
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        cfg = Config

        # QoS for PX4 topics
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # ── Publishers ──────────────────────────────────────────
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # ── Subscribers ─────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self._on_status, px4_qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._on_attitude, px4_qos)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land, px4_qos)
        self.create_subscription(
            Image, '/gimbal_camera', self._on_camera, 10)

        # ── Camera / ArUco setup ─────────────────────────────────
        self.bridge = CvBridge()
        aruco_dict = aruco.getPredefinedDictionary(cfg.ARUCO_DICT)
        params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, params)

        # ── MAVLink connection (pymavlink) ───────────────────────
        self.mav_conn = None
        self._init_mavlink()

        # ── State ────────────────────────────────────────────────
        self.state           = 'INIT'
        self.pos             = np.zeros(3)      # NED [x, y, z]
        self.q_att           = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion w,x,y,z
        self.armed           = False
        self.offboard_active = False
        self.landed          = True
        self.warmup_count    = 0

        # Setpoint (NED)
        self.sp     = np.array([0.0, 0.0, 0.0])
        self.sp_yaw = 0.0

        # Vision
        self.marker_detected = False
        self.marker_tvec: Optional[np.ndarray] = None  # translation in camera frame
        self.marker_rvec: Optional[np.ndarray] = None  # rotation vector
        self.marker_center_px: Optional[np.ndarray] = None
        self.last_detection_time = 0.0
        self.target_abs: Optional[np.ndarray] = None
        self.target_rel_xy: Optional[np.ndarray] = None
        self.target_rel_norm = float('inf')

        # Timers
        self._search_start   = None
        self._align_start    = None
        self._target_lost_start = None
        self._target_lost_from_state = None
        self._reacquire_start = None
        self._gimbal_set     = False
        self._gimbal_control_configured = False
        self._gimbal_cmd_failures = 0
        self._precland_sent  = False
        self._target_counter = 0
        self._centered_count = 0
        self._search_attempts = 0
        self._descent_z_sp = -cfg.CRUISE_ALT
        self._land_cmd_sent = False
        self._land_cmd_time = None
        self._disarm_retry_counter = 0
        self._force_disarm_sent = False
        self._land_recovery_count = 0
        self._allow_known_pad_descent = False

        # Control loop
        self.timer = self.create_timer(1.0 / cfg.CTRL_HZ, self._loop)
        self.get_logger().info('🎯 ArucoPrecisionLander ready — state: INIT')
        self.get_logger().info(f'   Marker search position: x={cfg.SEARCH_POS[0]}m, y={cfg.SEARCH_POS[1]}m')
        self.get_logger().info(f'   Cruise altitude: {cfg.CRUISE_ALT}m')

    # ══════════════════════════════════════════════════════════════
    #  MAVLINK SETUP
    # ══════════════════════════════════════════════════════════════

    def _init_mavlink(self):
        """Initialize pymavlink connection to PX4 SITL"""
        try:
            os.environ.setdefault('MAVLINK20', '1')
            from pymavlink import mavutil
            self.mav_conn = mavutil.mavlink_connection(
                f'udpout:{Config.MAVLINK_HOST}:{Config.MAVLINK_PORT}',
                dialect='common')
            self.get_logger().info(
                f'✅ MAVLink connected: udpout:{Config.MAVLINK_HOST}:{Config.MAVLINK_PORT}')
        except ImportError:
            self.get_logger().error(
                '❌ pymavlink not installed! Run: pip3 install pymavlink')
            self.mav_conn = None
        except Exception as e:
            self.get_logger().error(f'❌ MAVLink connection failed: {e}')
            self.mav_conn = None

    # ══════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════════════════════════

    def _on_position(self, msg: VehicleLocalPosition):
        self.pos = np.array([msg.x, msg.y, msg.z])

    def _on_status(self, msg: VehicleStatus):
        self.armed           = (msg.arming_state == 2)
        self.offboard_active = (msg.nav_state == 14)

    def _on_attitude(self, msg: VehicleAttitude):
        self.q_att = np.array(msg.q)  # [w, x, y, z]

    def _on_land(self, msg: VehicleLandDetected):
        self.landed = msg.landed

    def _on_camera(self, msg: Image):
        """Detect ArUco marker and compute pose"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                # Use first detected marker
                rvec, tvec = self._estimate_marker_pose(corners[0])
                self.marker_tvec = tvec  # [x, y, z] in camera frame
                self.marker_rvec = rvec
                self.marker_center_px = corners[0].reshape((-1, 2)).mean(axis=0)
                self.marker_detected = True
                self.last_detection_time = time.time()
            else:
                # Keep the last detection briefly so single-frame dropouts do not
                # interrupt the precision landing descent.
                if time.time() - self.last_detection_time > Config.MARKER_LOST_TIMEOUT:
                    self.marker_detected = False
                    self.marker_tvec = None
                    self.marker_rvec = None
                    self.marker_center_px = None
        except Exception as e:
            self.get_logger().warn(f'[Camera] {e}', throttle_duration_sec=2.0)

    def _estimate_marker_pose(self, corners: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate marker pose without relying on OpenCV contrib-only helpers."""
        half = Config.MARKER_SIZE / 2.0
        object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        image_points = corners.reshape((4, 2)).astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            Config.CAM_MTX,
            Config.DIST,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            raise RuntimeError('solvePnP failed for detected ArUco marker')

        return rvec.reshape(3), tvec.reshape(3)

    # ══════════════════════════════════════════════════════════════
    #  CONTROL LOOP
    # ══════════════════════════════════════════════════════════════

    def _loop(self):
        # Always publish offboard heartbeat
        self._pub_offboard()

        # State machine dispatch
        handler = getattr(self, f'_state_{self.state.lower()}', None)
        if handler:
            handler()
        else:
            self.get_logger().error(f'Unknown state: {self.state}')

        # Publish setpoint (needed while in offboard mode)
        self._pub_setpoint()

    # ══════════════════════════════════════════════════════════════
    #  STATE HANDLERS
    # ══════════════════════════════════════════════════════════════

    def _state_init(self):
        """Warm-up: stream offboard setpoints for 2s before arming"""
        self.sp = np.array([0.0, 0.0, 0.0])
        self.warmup_count += 1
        if self.warmup_count >= Config.CTRL_HZ * Config.WARMUP_SEC:
            self.warmup_count = 0
            self._transition('TAKEOFF')

    def _state_takeoff(self):
        """Climb to cruise altitude"""
        if not (self.armed and self.offboard_active):
            self.warmup_count += 1
            if self.warmup_count % 10 == 0:  # retry every 0.5 seconds
                if not self.offboard_active:
                    self.get_logger().info('Switching to Offboard mode...')
                    self._cmd_set_offboard()
                if not self.armed:
                    self.get_logger().info('Arming...')
                    self._cmd_arm()
            return  # Wait for arm + offboard

        self.sp = np.array([0.0, 0.0, -Config.CRUISE_ALT])
        alt_err = abs(self.pos[2] - (-Config.CRUISE_ALT))
        if alt_err < 0.3:
            self.get_logger().info(
                f'✅ Takeoff complete at {-self.pos[2]:.1f}m — pitching gimbal down')
            self._transition('GIMBAL_DOWN')

    def _state_fly_to_search(self):
        """Fly horizontally to approximate marker location"""
        cfg = Config
        self.sp = np.array([cfg.SEARCH_POS[0], cfg.SEARCH_POS[1], -cfg.CRUISE_ALT])

        dist = np.linalg.norm(self.pos[:2] - np.array(cfg.SEARCH_POS))
        if dist < 0.5:
            self.get_logger().info('📍 Arrived at search area — starting ArUco search')
            self._search_start = time.time()
            self._transition('SEARCH')

    def _state_gimbal_down(self):
        """Pitch gimbal to look straight down (-90°)"""
        self.sp = np.array([0.0, 0.0, -Config.CRUISE_ALT])
        if self.warmup_count % 5 == 0:
            self._cmd_gimbal_pitch(-1.5708)

        if not self._gimbal_set and self.warmup_count == 0:
            self.get_logger().info('📷 Commanding gimbal pitch to -90°')

        self.warmup_count += 1
        if self.warmup_count > Config.CTRL_HZ * Config.GIMBAL_SETTLE_SEC:
            self.warmup_count = 0
            self._gimbal_set = True
            self.get_logger().info('📷 Gimbal settle complete — flying to search area')
            self._transition('FLY_TO_SEARCH')

    def _state_search(self):
        """Hold position and wait for ArUco detection"""
        cfg = Config
        self.sp = np.array([cfg.SEARCH_POS[0], cfg.SEARCH_POS[1], -cfg.CRUISE_ALT])

        if self.marker_detected:
            self.get_logger().info(
                f'🎯 ArUco DETECTED! tvec=[{self.marker_tvec[0]:.2f}, '
                f'{self.marker_tvec[1]:.2f}, {self.marker_tvec[2]:.2f}]')
            self.target_abs = self._pad_target_abs()
            self._update_target_from_vision()
            self._align_start = time.time()
            self._target_counter = 0
            self._centered_count = 0
            self._transition('HORIZONTAL_APPROACH')
            return

        # Timeout check
        elapsed = time.time() - self._search_start
        if elapsed > cfg.SEARCH_TIMEOUT:
            self.get_logger().warn('⏰ Search timeout — landing normally')
            self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._transition('DONE')

    def _state_horizontal_approach(self):
        """PX4 phase 1: move horizontally over the target while holding altitude."""
        cfg = Config

        if not self._marker_recent():
            self._start_target_lost('target lost during horizontal approach')
            return

        if not self._update_target_from_vision():
            self._start_target_lost('vision correction unavailable during horizontal approach')
            return

        self._send_current_landing_target()
        self.sp = self._step_setpoint_to_target(-cfg.CRUISE_ALT, cfg.MAX_APPROACH_STEP)
        target_error = self._target_xy_error()

        if self._target_counter % cfg.CTRL_HZ == 0:
            self.get_logger().info(
                f'🧭 Horizontal approach: visual_error={self.target_rel_norm:.2f}m, '
                f'target_error={target_error:.2f}m, '
                f'target=[{self.target_abs[0]:.2f}, {self.target_abs[1]:.2f}]')

        self._target_counter += 1

        if self.target_rel_norm < cfg.PLD_HACC_RAD:
            self._centered_count += 1
        else:
            self._centered_count = 0

        if self._centered_count >= cfg.CENTER_CONFIRM_COUNT:
            self.get_logger().info('✅ PX4 horizontal acceptance reached — descending over target')
            self._target_counter = 0
            self._centered_count = 0
            self._descent_z_sp = float(self.pos[2])
            self._transition('DESCEND_OVER_TARGET')
            return

        if time.time() - self._align_start > cfg.ALIGN_TIMEOUT:
            self.get_logger().warn('Horizontal approach timeout — starting required-mode target search')
            self._target_counter = 0
            self._start_target_lost('horizontal approach timed out')

    def _state_descend_over_target(self):
        """PX4 phase 2: descend while staying centered over the target."""
        cfg = Config

        if not self._marker_recent():
            self._start_target_lost('target lost during descent', search_required=False)
            return

        if not self._update_target_from_vision():
            self._start_target_lost('vision correction unavailable during descent', search_required=False)
            return

        self._send_current_landing_target()

        if self.target_rel_norm > cfg.DESCENT_HACC_RAD:
            self.get_logger().warn(
                f'Descent target drift: visual_error={self.target_rel_norm:.2f}m — returning to horizontal approach')
            self._target_counter = 0
            self._align_start = time.time()
            self._centered_count = 0
            self._transition('HORIZONTAL_APPROACH')
            return

        final_approach_z = -cfg.PLD_FAPPR_ALT
        self._descent_z_sp = min(
            final_approach_z,
            self._descent_z_sp + cfg.DESCENT_RATE / cfg.CTRL_HZ
        )
        self.sp = self._step_setpoint_to_target(self._descent_z_sp, cfg.MAX_DESCENT_STEP)

        if self._target_counter % cfg.CTRL_HZ == 0:
            self.get_logger().info(
                f'🛬 Descend over target: alt={-self.pos[2]:.2f}m, '
                f'target_alt={-self._descent_z_sp:.2f}m, '
                f'visual_error={self.target_rel_norm:.2f}m')

        self._target_counter += 1

        if -self.pos[2] <= cfg.PLD_FAPPR_ALT + 0.05:
            self.get_logger().info('🛬 PX4 final-approach altitude reached')
            self._target_counter = 0
            self._transition('FINAL_APPROACH')

    def _state_final_approach(self):
        """PX4 phase 3: below PLD_FAPPR_ALT, continue even if the target drops out."""
        cfg = Config

        if self._marker_recent():
            self._update_target_from_vision()
            self._send_current_landing_target()
        elif self.target_abs is None:
            self.target_abs = self._pad_target_abs()

        final_z = -cfg.FINAL_ALT
        self._descent_z_sp = min(
            final_z,
            self._descent_z_sp + cfg.DESCENT_RATE / cfg.CTRL_HZ
        )
        self.sp = self._step_setpoint_to_target(self._descent_z_sp, cfg.MAX_DESCENT_STEP)

        if self._target_counter % cfg.CTRL_HZ == 0:
            marker_state = 'visible' if self._marker_recent() else 'lost'
            self.get_logger().info(
                f'🛬 Final approach: alt={-self.pos[2]:.2f}m, marker={marker_state}')

        self._target_counter += 1

        if self.pos[2] > -(cfg.FINAL_ALT + 0.05):
            self.get_logger().info('🛬 0.1m reached — switching to normal land/disarm')
            self._transition('LAND')

    def _state_target_lost(self):
        """PX4 required-mode search behaviour after PLD_BTOUT target loss."""
        cfg = Config
        if self._target_lost_start is None:
            self._target_lost_start = time.time()

        if self.target_abs is None:
            self.target_abs = self._pad_target_abs()

        if self._marker_recent():
            if self._target_lost_from_state == 'DESCEND_OVER_TARGET':
                self.get_logger().info('🎯 Target reacquired — continuing descent')
                self._target_counter = 0
                self._search_attempts = 0
                self._target_lost_start = None
                self._target_lost_from_state = None
                self._transition('DESCEND_OVER_TARGET')
                return

            self.get_logger().info('🎯 Target reacquired — restarting horizontal approach')
            self._target_counter = 0
            self._centered_count = 0
            self._search_attempts = 0
            self._target_lost_start = None
            self._target_lost_from_state = None
            self._align_start = time.time()
            self._transition('HORIZONTAL_APPROACH')
            return

        elapsed = time.time() - self._target_lost_start

        if self._target_lost_from_state == 'DESCEND_OVER_TARGET':
            if elapsed < cfg.DESCENT_LOSS_GRACE:
                self._continue_descent_without_target(elapsed)
                return

            self.get_logger().warn(
                'Target still lost during descent grace — landing on last good target')
            self._transition('LAND')
            return

        if elapsed < cfg.PLD_BTOUT:
            self.sp = self._step_setpoint_to_target(float(self.pos[2]), cfg.MAX_DESCENT_STEP)
            return

        search_alt = -cfg.PLD_SRCH_ALT
        self.sp = self._step_setpoint_to_target(search_alt, cfg.MAX_APPROACH_STEP)

        if self._target_counter % cfg.CTRL_HZ == 0:
            self.get_logger().info(
                f'🔁 PX4-style target search: attempt={self._search_attempts}/{cfg.PLD_MAX_SRCH}, '
                f'elapsed={elapsed:.1f}s')

        self._target_counter += 1

        if time.time() - self._target_lost_start > cfg.SEARCH_TIMEOUT:
            if self._search_attempts >= cfg.PLD_MAX_SRCH:
                self.get_logger().warn('Precision target not found after max search attempts — normal landing')
                self._transition('LAND')
                return

            self._search_attempts += 1
            self._target_lost_start = time.time()
            self._target_lost_from_state = 'HORIZONTAL_APPROACH'
            self._target_counter = 0
            self.get_logger().warn('Search timed out — repeating required-mode search')

    def _state_land(self):
        """Let PX4 finish touchdown once the vehicle is very close to the pad."""
        cfg = Config
        if self._marker_recent():
            self._update_target_from_vision()
            self._send_current_landing_target()
        elif self.target_abs is None:
            self.target_abs = self._pad_target_abs()

        self.sp = np.array([self.target_abs[0], self.target_abs[1], -cfg.FINAL_ALT])

        if not self._land_cmd_sent:
            self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._land_cmd_sent = True
            self._land_cmd_time = time.time()

        land_elapsed = time.time() - self._land_cmd_time if self._land_cmd_time else 0.0
        near_ground = self.pos[2] > -(cfg.FINAL_ALT + 0.08)
        should_disarm = self.landed or (land_elapsed > 3.0 and near_ground)

        if should_disarm and not self.armed:
            self.get_logger().info('✅ LANDING COMPLETE!')
            self._transition('DONE')
        elif should_disarm:
            if self._disarm_retry_counter % cfg.CTRL_HZ == 0:
                if (
                    not self.landed
                    and land_elapsed > cfg.FORCE_DISARM_DELAY
                    and not self._force_disarm_sent
                ):
                    self.get_logger().warn(
                        'PX4 still reports not landed — force disarming at ground height')
                    self._cmd_force_disarm()
                    self._force_disarm_sent = True
                else:
                    reason = 'land detector' if self.landed else 'low altitude fallback'
                    self.get_logger().info(f'🛬 Touchdown detected by {reason} — disarming...')
                    self._cmd_disarm()
            self._disarm_retry_counter += 1

    def _state_done(self):
        """Mission complete"""
        pass

    # ══════════════════════════════════════════════════════════════
    #  MAVLINK LANDING TARGET
    # ══════════════════════════════════════════════════════════════

    def _pad_target_abs(self) -> np.ndarray:
        return np.array([Config.SEARCH_POS[0], Config.SEARCH_POS[1], Config.LANDING_TARGET_Z])

    def _marker_recent(self) -> bool:
        return self.marker_detected and (time.time() - self.last_detection_time) < Config.MARKER_LOST_TIMEOUT

    def _target_xy_error(self) -> float:
        if self.target_abs is None:
            return float('inf')
        return float(np.linalg.norm(self.pos[:2] - self.target_abs[:2]))

    def _start_target_lost(self, reason: str, search_required: bool = True):
        cfg = Config
        if self.state != 'TARGET_LOST':
            if search_required:
                suffix = f'waiting PLD_BTOUT={cfg.PLD_BTOUT:.1f}s'
            else:
                suffix = f'continuing on last target for {cfg.DESCENT_LOSS_GRACE:.1f}s'
            self.get_logger().warn(f'{reason} — {suffix}')
            self._target_lost_start = time.time()
            self._target_lost_from_state = self.state
            self._target_counter = 0
            if search_required:
                self._search_attempts += 1
            self._transition('TARGET_LOST')
            return

        if self._target_lost_start is None:
            self._target_lost_start = time.time()

    def _continue_descent_without_target(self, elapsed: float):
        cfg = Config
        final_approach_z = -cfg.PLD_FAPPR_ALT
        self._descent_z_sp = min(
            final_approach_z,
            self._descent_z_sp + cfg.LOST_TARGET_DESCENT_RATE / cfg.CTRL_HZ
        )
        self.sp = self._step_setpoint_to_target(self._descent_z_sp, cfg.MAX_DESCENT_STEP)

        if self._target_counter % cfg.CTRL_HZ == 0:
            self.get_logger().info(
                f'🛬 Target temporarily lost: continuing descent on last target, '
                f'alt={-self.pos[2]:.2f}m, elapsed={elapsed:.1f}s')

        self._target_counter += 1

        if -self.pos[2] <= cfg.PLD_FAPPR_ALT + 0.05:
            self.get_logger().info('🛬 Final-approach altitude reached without target')
            self._target_lost_start = None
            self._target_lost_from_state = None
            self._target_counter = 0
            self._transition('FINAL_APPROACH')

    def _step_setpoint_to_target(self, target_z: float, max_step: float) -> np.ndarray:
        if self.target_abs is None:
            self.target_abs = self._pad_target_abs()

        delta_xy = self.target_abs[:2] - self.pos[:2]
        dist = float(np.linalg.norm(delta_xy))
        if dist > max_step:
            target_xy = self.pos[:2] + delta_xy * (max_step / dist)
        else:
            target_xy = self.target_abs[:2]

        return np.array([target_xy[0], target_xy[1], target_z])

    def _marker_image_rel_ned(self) -> Optional[np.ndarray]:
        """Estimate marker ground offset from image-center error for a downward camera."""
        if self.marker_center_px is None:
            return None

        cfg = Config
        alt = max(-float(self.pos[2]), cfg.FINAL_ALT)
        vfov = 2.0 * math.atan(math.tan(cfg.HFOV / 2.0) * (cfg.IMG_H / cfg.IMG_W))
        meters_per_px_x = (2.0 * alt * math.tan(cfg.HFOV / 2.0)) / cfg.IMG_W
        meters_per_px_y = (2.0 * alt * math.tan(vfov / 2.0)) / cfg.IMG_H

        dx_px = float(self.marker_center_px[0] - cfg.CAM_CX)
        dy_px = float(self.marker_center_px[1] - cfg.CAM_CY)

        # Image x is camera-right/east; image y is camera-down/south for a nadir view.
        rel_north = -dy_px * meters_per_px_y
        rel_east = dx_px * meters_per_px_x
        rel = np.array([rel_north, rel_east], dtype=float)

        norm = float(np.linalg.norm(rel))
        if norm > cfg.MAX_VISUAL_CORRECTION:
            self.get_logger().warn(
                f'Vision correction rejected: {norm:.2f}m from image center',
                throttle_duration_sec=1.0)
            return None

        return rel

    def _update_target_from_vision(self) -> bool:
        """Update absolute landing target from bounded visual marker offset."""
        cfg = Config
        rel_xy = self._marker_image_rel_ned()
        if rel_xy is None:
            if self.target_abs is None:
                self.target_abs = self._pad_target_abs()
            self.target_rel_xy = None
            self.target_rel_norm = float('inf')
            return False

        self.target_rel_xy = rel_xy
        self.target_rel_norm = float(np.linalg.norm(rel_xy))

        measured_target_xy = self.pos[:2] + rel_xy
        known_xy = np.array(cfg.SEARCH_POS, dtype=float)
        delta = measured_target_xy - known_xy
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > cfg.MAX_TARGET_OFFSET_FROM_SEARCH:
            measured_target_xy = known_xy + delta * (cfg.MAX_TARGET_OFFSET_FROM_SEARCH / delta_norm)

        if self.target_abs is None:
            target_xy = measured_target_xy
        else:
            target_xy = (
                (1.0 - cfg.VISION_TARGET_ALPHA) * self.target_abs[:2]
                + cfg.VISION_TARGET_ALPHA * measured_target_xy
            )

        self.target_abs = np.array([target_xy[0], target_xy[1], cfg.LANDING_TARGET_Z])
        return True

    def _send_current_landing_target(self):
        if self.target_abs is None:
            return

        if self.target_rel_xy is None:
            rel_x = float(self.target_abs[0] - self.pos[0])
            rel_y = float(self.target_abs[1] - self.pos[1])
        else:
            rel_x = float(self.target_rel_xy[0])
            rel_y = float(self.target_rel_xy[1])

        rel_z = max(-float(self.pos[2]), Config.FINAL_ALT)
        self._send_landing_target(rel_x, rel_y, rel_z)

    def _marker_rel_ned(self) -> Tuple[float, float, float]:
        """Convert marker translation from camera frame into vehicle-carried NED."""
        # Camera frame from OpenCV pose: x=right, y=down, z=forward into scene.
        # With the gimbal pitched down: camera x -> east, camera y -> -north, camera z -> down.
        return (
            float(-self.marker_tvec[1]),
            float(self.marker_tvec[0]),
            float(self.marker_tvec[2]),
        )

    def _send_landing_target(self, rel_x: float, rel_y: float, rel_z: float):
        """Send MAVLink LANDING_TARGET message to PX4"""
        if self.mav_conn is None:
            return

        try:
            # LANDING_TARGET message
            # frame = MAV_FRAME_LOCAL_NED (1)
            # position_valid = 1
            # PX4 interprets x, y, z in MAV_FRAME_LOCAL_NED as absolute local-NED target position.
            if self.target_abs is not None:
                target_x = float(self.target_abs[0])
                target_y = float(self.target_abs[1])
                target_z = float(self.target_abs[2])
            else:
                target_x = float(self.pos[0] + rel_x)
                target_y = float(self.pos[1] + rel_y)
                target_z = float(self.pos[2] + rel_z)
            self.mav_conn.mav.landing_target_send(
                int(time.time() * 1e6),
                0,
                1,  # MAV_FRAME_LOCAL_NED
                0.0,
                0.0,
                float(np.sqrt(rel_x**2 + rel_y**2 + rel_z**2)),
                Config.MARKER_SIZE,
                Config.MARKER_SIZE,
                target_x,
                target_y,
                target_z,
                [1.0, 0.0, 0.0, 0.0],
                2,  # LANDING_TARGET_TYPE_VISION_FIDUCIAL
                1,
            )
        except Exception as e:
            self.get_logger().warn(f'MAVLink send error: {e}', throttle_duration_sec=2.0)

    # ══════════════════════════════════════════════════════════════
    #  GIMBAL CONTROL
    # ══════════════════════════════════════════════════════════════

    def _cmd_gimbal_pitch(self, pitch_rad: float):
        """Pitch gimbal through PX4's gimbal manager/mount command path."""
        pitch_deg = math.degrees(pitch_rad)

        if not self._gimbal_control_configured:
            self._cmd(
                VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                p1=1.0, p2=1.0,
                target_component=0,
            )
            self._gimbal_control_configured = True

        self._cmd(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            p1=pitch_deg, p2=0.0, p3=math.nan, p4=math.nan, p5=0.0,
            target_component=0,
        )
        self._cmd(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
            p1=pitch_deg, p2=0.0, p3=0.0,
            p7=float(VehicleCommand.VEHICLE_MOUNT_MODE_MAVLINK_TARGETING),
            target_component=0,
        )

    def _send_px4_shell(self, command: str):
        """Send command to PX4 MAVLink shell"""
        if self.mav_conn is None:
            return
        try:
            # Use MAV_CMD_DO_SEND_BANNER workaround or direct shell
            # Simpler: use subprocess to send via commander
            self.get_logger().info(f'PX4 shell: {command}')
        except Exception as e:
            self.get_logger().warn(f'PX4 shell failed: {e}')

    # ══════════════════════════════════════════════════════════════
    #  PX4 COMMANDS
    # ══════════════════════════════════════════════════════════════

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

    def _cmd_force_disarm(self):
        self._cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            p1=0.0,
            p2=Config.FORCE_DISARM_MAGIC,
        )

    def _cmd(self, command: int, p1: float = 0.0, p2: float = 0.0,
             p3: float = 0.0, p4: float = 0.0, p5: float = 0.0,
             p6: float = 0.0, p7: float = 0.0,
             target_system: int = 1, target_component: int = 1):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = p1
        msg.param2           = p2
        msg.param3           = p3
        msg.param4           = p4
        msg.param5           = p5
        msg.param6           = p6
        msg.param7           = p7
        msg.target_system    = target_system
        msg.target_component = target_component
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._ts()
        self.pub_cmd.publish(msg)

    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state

    def _ts(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


# ══════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPrecisionLander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
