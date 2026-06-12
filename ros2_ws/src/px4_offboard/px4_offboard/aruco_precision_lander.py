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
from typing import Dict, List, Optional, Tuple

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
    HFOV    = 1.2       # radians (~68.7°)
    # Focal length: fx = (IMG_W/2) / tan(HFOV/2)
    CAM_FX  = (IMG_W / 2) / math.tan(HFOV / 2)  # ≈ 935
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

    # ── Nested ArUco ──────────────────────────────────────────
    NESTED_TAG_IDS = (10, 11, 12)          # outer, middle, inner
    NESTED_MARKER_SIZES = (1.00, 0.22, 0.08)
    NESTED_CRUISE_ALT = 5.0               # current Gazebo texture is reliable near this altitude
    NESTED_MIDDLE_SWITCH_IN_Z = 2.5
    NESTED_MIDDLE_SWITCH_OUT_Z = 3.0
    NESTED_INNER_SWITCH_IN_Z = 1.0
    NESTED_INNER_SWITCH_OUT_Z = 1.3
    NESTED_SWITCH_STABLE_SEC = 0.5
    NESTED_SWITCH_STABLE_FRAMES = 5
    NESTED_LOST_HOLD_SEC = 0.8
    NESTED_VISUAL_ALPHA = 0.35
    NESTED_ALIGN_HACC_RAD = 0.35
    NESTED_ALIGN_CONFIRM_COUNT = 8
    NESTED_DESCENT_HACC_RAD = 0.55
    NESTED_FINAL_ALT = 0.30
    NESTED_SERVO_GAIN = 0.85
    NESTED_MAX_ALIGN_STEP = 0.45
    NESTED_MAX_DESCENT_STEP = 0.30
    NESTED_TARGET_LOSS_GRACE = 1.5
    # Fractal tracker pose uses camera optical axes: x=image-right, y=image-down.
    # With the gimbal locked downward/yaw-stabilized, image-right maps to ENU East
    # and image-down maps to ENU South, so North is -camera_y.
    NESTED_CAMERA_X_TO_BODY_EAST_SIGN = 1.0
    NESTED_CAMERA_Y_TO_BODY_NORTH_SIGN = -1.0
    # "local": camera x/y already follow local ENU, e.g. yaw-stabilized gimbal.
    # "body": rotate camera x/y by vehicle yaw before becoming local ENU.
    NESTED_CAMERA_YAW_FRAME = 'local'

    # ── Flight ────────────────────────────────────────────────
    CRUISE_ALT    = 5.0     # m — search altitude
    SEARCH_FRAME  = 'enu'   # search_x/search_y are east/north by default
    SEARCH_POS    = (3.0, 2.0)  # approximate marker location in SEARCH_FRAME

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
    SEARCH_DIAG_INTERVAL = 2.0
    TARGET_SEND_HZ = 10    # Hz for LANDING_TARGET messages
    LANDING_TARGET_Z = 0.0
    DESCENT_RATE = 0.45     # m/s
    FINAL_ALT = 0.10        # m above ground before switching to land mode
    FORCE_DISARM_DELAY = 8.0
    FORCE_DISARM_MAGIC = 21196.0
    LAND_WAIT_WARN_INTERVAL = 2.0
    ALIGN_TIMEOUT = 18.0

    # PX4 precision-landing equivalents. See PX4 PLD_HACC_RAD, PLD_BTOUT,
    # PLD_FAPPR_ALT, PLD_MAX_SRCH, and PLD_SRCH_ALT.
    PLD_HACC_RAD = 0.25
    PLD_BTOUT = 2.0
    PLD_FAPPR_ALT = 0.10
    PLD_MAX_SRCH = 3
    PLD_SRCH_ALT = 5.0
    DESCENT_HACC_RAD = 0.35
    NESTED_DESCENT_HACC_RAD = 0.85
    DESCENT_DRIFT_CONFIRM_COUNT = 8
    CENTER_CONFIRM_COUNT = 8
    DESCENT_LOSS_GRACE = 8.0
    LOST_TARGET_DESCENT_RATE = 0.25

    MARKER_LOST_TIMEOUT = 1.5
    TARGET_CURRENT_TIMEOUT = 1.2
    VISION_REUSE_TIMEOUT = 1.2
    VISION_TARGET_ALPHA = 0.25
    MAX_TARGET_OFFSET_FROM_SEARCH = 1.5
    NESTED_MAX_TARGET_OFFSET_FROM_SEARCH = 0.75
    MAX_VISUAL_CORRECTION = 2.5
    MAX_APPROACH_STEP = 0.65
    MAX_DESCENT_STEP = 0.35


# ══════════════════════════════════════════════════════════════════
#  MAIN NODE
# ══════════════════════════════════════════════════════════════════

class ArucoPrecisionLander(Node):

    def __init__(self, node_name: str = 'aruco_precision_lander', nested_mode_default: bool = False):
        super().__init__(node_name)
        cfg = Config
        self.declare_parameter('allow_force_disarm', False)
        self.declare_parameter('nested_mode', nested_mode_default)
        self.nested_mode = bool(self.get_parameter('nested_mode').value)
        cruise_default = cfg.NESTED_CRUISE_ALT if self.nested_mode else cfg.CRUISE_ALT
        self.declare_parameter('cruise_alt', cruise_default)
        self.declare_parameter('pld_srch_alt', cruise_default)
        self.declare_parameter('search_x', cfg.SEARCH_POS[0])
        self.declare_parameter('search_y', cfg.SEARCH_POS[1])
        self.declare_parameter('search_frame', cfg.SEARCH_FRAME)
        self.declare_parameter('camera_topic', '/gimbal_camera')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_image_topic', '/landing/annotated_image')
        self.declare_parameter('show_debug_view', False)
        self.declare_parameter('nested_tag_ids', list(cfg.NESTED_TAG_IDS))
        self.declare_parameter('nested_marker_sizes_m', list(cfg.NESTED_MARKER_SIZES))
        self.declare_parameter('middle_switch_in_z', cfg.NESTED_MIDDLE_SWITCH_IN_Z)
        self.declare_parameter('middle_switch_out_z', cfg.NESTED_MIDDLE_SWITCH_OUT_Z)
        self.declare_parameter('inner_switch_in_z', cfg.NESTED_INNER_SWITCH_IN_Z)
        self.declare_parameter('inner_switch_out_z', cfg.NESTED_INNER_SWITCH_OUT_Z)
        self.declare_parameter('nested_switch_stable_sec', cfg.NESTED_SWITCH_STABLE_SEC)
        self.declare_parameter('nested_visual_alpha', cfg.NESTED_VISUAL_ALPHA)
        self.declare_parameter('nested_align_hacc_rad', cfg.NESTED_ALIGN_HACC_RAD)
        self.declare_parameter('nested_align_confirm_count', cfg.NESTED_ALIGN_CONFIRM_COUNT)
        self.declare_parameter('nested_descent_hacc_rad', cfg.NESTED_DESCENT_HACC_RAD)
        self.declare_parameter('nested_final_alt', cfg.NESTED_FINAL_ALT)
        self.declare_parameter('nested_servo_gain', cfg.NESTED_SERVO_GAIN)
        self.declare_parameter('nested_max_align_step', cfg.NESTED_MAX_ALIGN_STEP)
        self.declare_parameter('nested_max_descent_step', cfg.NESTED_MAX_DESCENT_STEP)
        self.declare_parameter('nested_target_loss_grace', cfg.NESTED_TARGET_LOSS_GRACE)
        self.declare_parameter('camera_x_to_body_east_sign', cfg.NESTED_CAMERA_X_TO_BODY_EAST_SIGN)
        self.declare_parameter('camera_y_to_body_north_sign', cfg.NESTED_CAMERA_Y_TO_BODY_NORTH_SIGN)
        self.declare_parameter('camera_yaw_frame', cfg.NESTED_CAMERA_YAW_FRAME)
        self.declare_parameter('use_cpp_tracker', True)
        self.declare_parameter('pose_topic', '/aruco_fractal_tracker/poses')

        self.allow_force_disarm = bool(self.get_parameter('allow_force_disarm').value)
        cfg.CRUISE_ALT = float(self.get_parameter('cruise_alt').value)
        cfg.PLD_SRCH_ALT = float(self.get_parameter('pld_srch_alt').value)
        self.search_input_frame = str(self.get_parameter('search_frame').value).strip().lower()
        search_xy = np.array([
            float(self.get_parameter('search_x').value),
            float(self.get_parameter('search_y').value),
        ], dtype=float)
        if self.search_input_frame == 'enu':
            self.search_enu_xy = search_xy
            self.search_ned_xy = self._enu_xy_to_ned_xy(search_xy)
        elif self.search_input_frame == 'ned':
            self.search_ned_xy = search_xy
            self.search_enu_xy = self._ned_xy_to_enu_xy(search_xy)
        else:
            raise ValueError("search_frame must be 'enu' or 'ned'")
        cfg.SEARCH_POS = (float(self.search_ned_xy[0]), float(self.search_ned_xy[1]))
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.publish_annotated_image = bool(self.get_parameter('publish_annotated_image').value)
        self.annotated_image_topic = str(self.get_parameter('annotated_image_topic').value)
        self.show_debug_view = bool(self.get_parameter('show_debug_view').value)
        if self.nested_mode:
            cfg.MAX_TARGET_OFFSET_FROM_SEARCH = cfg.NESTED_MAX_TARGET_OFFSET_FROM_SEARCH
            cfg.DESCENT_HACC_RAD = cfg.NESTED_DESCENT_HACC_RAD

        nested_ids = [int(marker_id) for marker_id in self.get_parameter('nested_tag_ids').value]
        nested_sizes = [float(size) for size in self.get_parameter('nested_marker_sizes_m').value]
        if len(nested_ids) != 3 or len(nested_sizes) != 3:
            raise ValueError('nested_tag_ids and nested_marker_sizes_m must each contain outer, middle, inner')
        self.nested_outer_id, self.nested_middle_id, self.nested_inner_id = nested_ids
        self.nested_marker_sizes = dict(zip(nested_ids, nested_sizes))
        self.nested_tag_names = {
            self.nested_outer_id: 'outer',
            self.nested_middle_id: 'middle',
            self.nested_inner_id: 'inner',
        }
        self.middle_switch_in_z = float(self.get_parameter('middle_switch_in_z').value)
        self.middle_switch_out_z = float(self.get_parameter('middle_switch_out_z').value)
        self.inner_switch_in_z = float(self.get_parameter('inner_switch_in_z').value)
        self.inner_switch_out_z = float(self.get_parameter('inner_switch_out_z').value)
        self.nested_switch_stable_sec = float(self.get_parameter('nested_switch_stable_sec').value)
        self.nested_visual_alpha = float(self.get_parameter('nested_visual_alpha').value)
        self.nested_align_hacc_rad = float(self.get_parameter('nested_align_hacc_rad').value)
        self.nested_align_confirm_count = int(self.get_parameter('nested_align_confirm_count').value)
        self.nested_descent_hacc_rad = float(self.get_parameter('nested_descent_hacc_rad').value)
        self.nested_final_alt = float(self.get_parameter('nested_final_alt').value)
        self.nested_servo_gain = float(self.get_parameter('nested_servo_gain').value)
        self.nested_max_align_step = float(self.get_parameter('nested_max_align_step').value)
        self.nested_max_descent_step = float(self.get_parameter('nested_max_descent_step').value)
        self.nested_target_loss_grace = float(self.get_parameter('nested_target_loss_grace').value)
        self.camera_x_to_body_east_sign = float(self.get_parameter('camera_x_to_body_east_sign').value)
        self.camera_y_to_body_north_sign = float(self.get_parameter('camera_y_to_body_north_sign').value)
        self.camera_yaw_frame = str(self.get_parameter('camera_yaw_frame').value).strip().lower()
        if self.camera_yaw_frame not in ('body', 'local'):
            raise ValueError("camera_yaw_frame must be 'body' or 'local'")
        self.use_cpp_tracker = bool(self.get_parameter('use_cpp_tracker').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)

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
        self.pub_annotated = None
        if self.publish_annotated_image:
            self.pub_annotated = self.create_publisher(Image, self.annotated_image_topic, 10)

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
        if self.nested_mode and self.use_cpp_tracker:
            from geometry_msgs.msg import PoseStamped
            self.create_subscription(
                PoseStamped, self.pose_topic, self._on_pose_stamped, 10)
        else:
            self.create_subscription(
                Image, self.camera_topic, self._on_camera, 10)

        # ── Camera / ArUco setup ─────────────────────────────────
        self.bridge = CvBridge()
        aruco_dict = aruco.getPredefinedDictionary(cfg.ARUCO_DICT)
        params = aruco.DetectorParameters()
        # Tune params for small and blurry marker detection at altitude
        params.minMarkerPerimeterRate = 0.010     # default 0.03
        params.maxMarkerPerimeterRate = 4.0
        params.polygonalApproxAccuracyRate = 0.05
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53     # default 23
        params.adaptiveThreshWinSizeStep = 8     # default 10
        params.adaptiveThreshConstant = 7
        params.minCornerDistanceRate = 0.02      # default 0.05
        params.minDistanceToBorder = 1
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        # Companion-friendly parameters (detect tiny markers without upscale)
        params.errorCorrectionRate = 0.9          # default 0.6
        params.perspectiveRemovePixelPerCell = 2  # default 4
        params.minSideLengthCanonicalImg = 16     # default 32

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
        self.ground_contact  = False
        self.maybe_landed    = False
        self.at_rest         = False
        self.warmup_count    = 0

        # Control logic is ENU-first. PX4 still requires NED at the
        # TrajectorySetpoint boundary, so self.sp stores that final adapter value.
        self.sp     = np.array([0.0, 0.0, 0.0])
        self.sp_yaw = 0.0

        # Vision
        self.marker_detected = False
        self.marker_tvec: Optional[np.ndarray] = None  # translation in camera frame
        self.marker_rvec: Optional[np.ndarray] = None  # rotation vector
        self.marker_center_px: Optional[np.ndarray] = None
        self.marker_id: Optional[int] = None
        self.marker_size = cfg.MARKER_SIZE
        self.visible_marker_ids: List[int] = []
        self.detected_markers: Dict[int, dict] = {}
        self.nested_current_id: Optional[int] = None
        self.nested_candidate_id: Optional[int] = None
        self.nested_candidate_since: Optional[float] = None
        self.nested_seen_counts: Dict[int, int] = {}
        self.nested_last_seen: Dict[int, float] = {}
        self.nested_last_detections: Dict[int, dict] = {}
        self.last_detection_time = 0.0
        self._last_processed_detection_time = 0.0
        self.camera_frame_count = 0
        self.last_camera_frame_time = 0.0
        self.target_abs: Optional[np.ndarray] = None
        self.target_rel_xy: Optional[np.ndarray] = None
        self.target_rel_norm = float('inf')
        self.nested_visual_rel_raw: Optional[np.ndarray] = None
        self.nested_visual_rel_filtered: Optional[np.ndarray] = None
        self.nested_visual_camera_xy: Optional[np.ndarray] = None
        self.nested_visual_last_update = 0.0
        self.nested_last_good_xy = np.array(cfg.SEARCH_POS, dtype=float)
        self._debug_frame_saved = False

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
        self._last_land_wait_log = 0.0
        self._land_recovery_count = 0
        self._allow_known_pad_descent = False
        self._last_search_diag = 0.0
        self._descent_drift_count = 0
        self._last_frame_debug = 0.0

        # Control loop
        self.timer = self.create_timer(1.0 / cfg.CTRL_HZ, self._loop)
        mode = 'Fractal ArUco' if self.nested_mode and self.use_cpp_tracker else (
            'Nested ArUco' if self.nested_mode else 'ArUco'
        )
        self.get_logger().info(f'🎯 {mode} PrecisionLander ready — state: INIT')
        self.get_logger().info(
            f'   Marker search input ({self.search_input_frame}): '
            f'x={search_xy[0]:.2f}m, y={search_xy[1]:.2f}m')
        self.get_logger().info(
            f'   Search ENU east/north=[{self.search_enu_xy[0]:.2f}, {self.search_enu_xy[1]:.2f}]')
        self.get_logger().info(
            '   Controller frame: ENU east/north/up; PX4 adapter publishes NED setpoints')
        self.get_logger().info(f'   Cruise altitude: {cfg.CRUISE_ALT}m')
        if self.nested_mode:
            if self.use_cpp_tracker:
                self.get_logger().info(
                    f'   Fractal tracker pose topic: {self.pose_topic}; synthetic target id=99')
            else:
                self.get_logger().info(
                    f'   Nested IDs outer/middle/inner: '
                    f'{self.nested_outer_id}/{self.nested_middle_id}/{self.nested_inner_id}')
            self.get_logger().info(
                f'   Vision frame: tracker camera optical -> {self.camera_yaw_frame} ENU, '
                f'signs x/east={self.camera_x_to_body_east_sign:+.0f}, '
                f'y/north={self.camera_y_to_body_north_sign:+.0f}')
        if self.pub_annotated is not None:
            self.get_logger().info(f'   Annotated camera topic: {self.annotated_image_topic}')
        if self.show_debug_view:
            window_title = 'Fractal ArUco precision landing' if self.use_cpp_tracker else 'ArUco precision landing'
            cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_title, 960, 540)
            self._debug_window_title = window_title

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
        self.ground_contact = msg.ground_contact
        self.maybe_landed = msg.maybe_landed
        self.at_rest = msg.at_rest
    def _on_pose_stamped(self, msg) -> None:
        """Callback for external C++ tracker poses."""
        tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        rvec = np.zeros(3)

        self.marker_tvec = tvec
        self.marker_rvec = rvec
        self.marker_center_px = np.array([640.0, 360.0]) # dummy center for 1280x720
        self.marker_id = 99  # synthetic ID for fractal marker
        self.marker_size = 1.0
        self.marker_detected = True
        self.last_detection_time = time.time()
        self.visible_marker_ids = [99]
        self.camera_frame_count += 1
        self.last_camera_frame_time = time.time()

        now = time.time()
        if not hasattr(self, '_last_pose_log') or now - self._last_pose_log > 1.0:
            frame_id = msg.header.frame_id or '<empty>'
            self.get_logger().info(
                f"📥 Tracker pose frame={frame_id}: "
                f"camera_optical_tvec=[{tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}]"
            )
            self._last_pose_log = now

    def _on_camera(self, msg: Image):
        """Detect ArUco marker(s), choose the active target, and compute pose."""
        try:
            self.camera_frame_count += 1
            self.last_camera_frame_time = time.time()
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Save debug frame on first SEARCH frame
            if self.state == 'SEARCH' and not self._debug_frame_saved:
                save_path = '/home/ducanh/PX4/examples/gimbal_simulation/search_frame.png'
                try:
                    cv2.imwrite(save_path, frame)
                    self._debug_frame_saved = True
                    self.get_logger().info(f'📷 Saved debug search frame at altitude {abs(self.pos[2]):.2f}m to {save_path}')
                except Exception as save_err:
                    self.get_logger().warn(f'Failed to save debug search frame: {save_err}')

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)

            # No upscale fallback (for CPU efficiency on companion computer)
            detections = self._build_marker_detections(corners, ids)
            self.detected_markers = {int(det['id']): det for det in detections}
            self.visible_marker_ids = [] if ids is None else ids.flatten().astype(int).tolist()

            if self.nested_mode:
                selected = self._select_nested_detection(self.detected_markers)
            else:
                selected = detections[0] if detections else None

            self._publish_annotated_frame(msg, frame, detections, selected, rejected)

            if selected is not None:
                self._accept_marker_detection(selected)
            else:
                self._clear_marker_if_stale()
        except Exception as e:
            self.get_logger().warn(f'[Camera] {e}', throttle_duration_sec=2.0)

    def _estimate_marker_pose(
        self,
        corners: np.ndarray,
        marker_size: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate marker pose without relying on OpenCV contrib-only helpers."""
        half = (Config.MARKER_SIZE if marker_size is None else marker_size) / 2.0
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

    def _build_marker_detections(self, corners, ids) -> List[dict]:
        if ids is None:
            return []

        detections = []
        for marker_corners, raw_id in zip(corners, ids.flatten()):
            marker_id = int(raw_id)
            if self.nested_mode and marker_id not in self.nested_marker_sizes:
                continue

            marker_size = self.nested_marker_sizes.get(marker_id, Config.MARKER_SIZE)
            rvec, tvec = self._estimate_marker_pose(marker_corners, marker_size)
            center_px = marker_corners.reshape((-1, 2)).mean(axis=0)
            detections.append({
                'id': marker_id,
                'name': self.nested_tag_names.get(marker_id, 'aruco'),
                'size': marker_size,
                'corners': marker_corners,
                'rvec': rvec,
                'tvec': tvec,
                'center_px': center_px,
            })

        detections.sort(key=lambda det: int(det['id']))
        return detections

    def _select_nested_detection(self, detections_by_id: Dict[int, dict]) -> Optional[dict]:
        self._update_nested_visibility(detections_by_id)
        preferred_id = self._preferred_nested_id(detections_by_id)
        if preferred_id is None:
            self._clear_nested_candidate()
            return None

        active_id = self._select_nested_active_id(preferred_id, detections_by_id)
        if active_id is None:
            return None

        source_id = active_id if active_id in detections_by_id else preferred_id
        selected = dict(detections_by_id[source_id])
        selected['id'] = active_id
        selected['name'] = self.nested_tag_names.get(active_id, 'nested')
        selected['center_px'] = self._fused_nested_center(detections_by_id)
        selected['source_id'] = source_id
        return selected

    def _preferred_nested_id(self, detections_by_id: Dict[int, dict]) -> Optional[int]:
        """Accept any visible nested marker. Prefer smallest (most precise) first."""
        for marker_id in (self.nested_inner_id, self.nested_middle_id, self.nested_outer_id):
            if marker_id in detections_by_id:
                return marker_id
        return None

    def _nested_priority(self, marker_id: Optional[int]) -> int:
        order = {
            self.nested_inner_id: 0,
            self.nested_middle_id: 1,
            self.nested_outer_id: 2,
        }
        return order.get(marker_id, 99)

    def _update_nested_visibility(self, detections_by_id: Dict[int, dict]):
        now = time.time()
        for marker_id in (self.nested_inner_id, self.nested_middle_id, self.nested_outer_id):
            if marker_id in detections_by_id:
                self.nested_seen_counts[marker_id] = self.nested_seen_counts.get(marker_id, 0) + 1
                self.nested_last_seen[marker_id] = now
                self.nested_last_detections[marker_id] = dict(detections_by_id[marker_id])
            else:
                self.nested_seen_counts[marker_id] = 0

    def _nested_seen_stably(self, marker_id: int) -> bool:
        return self.nested_seen_counts.get(marker_id, 0) >= Config.NESTED_SWITCH_STABLE_FRAMES

    def _nested_recently_seen(self, marker_id: int) -> bool:
        last_seen = self.nested_last_seen.get(marker_id)
        return last_seen is not None and (time.time() - last_seen) <= Config.NESTED_LOST_HOLD_SEC

    def _select_nested_active_id(
        self,
        preferred_id: int,
        detections_by_id: Dict[int, dict],
    ) -> Optional[int]:
        current_id = self.nested_current_id

        if current_id is None:
            if self._nested_seen_stably(preferred_id):
                self._accept_nested_id(preferred_id)
                return preferred_id
            return None

        preferred_is_better = self._nested_priority(preferred_id) < self._nested_priority(current_id)
        if preferred_is_better:
            if self._nested_seen_stably(preferred_id):
                self._accept_nested_id(preferred_id)
                return preferred_id
            return current_id if self._nested_recently_seen(current_id) else preferred_id

        current_visible_or_held = (
            current_id in detections_by_id
            or self._nested_recently_seen(current_id)
        )
        if current_visible_or_held:
            return current_id

        if self._nested_seen_stably(preferred_id):
            self._accept_nested_id(preferred_id)
            return preferred_id

        return preferred_id

    def _fused_nested_center(self, detections_by_id: Dict[int, dict]) -> np.ndarray:
        centers = []
        weights = []
        for marker_id, detection in detections_by_id.items():
            centers.append(detection['center_px'])
            weights.append(3.0 - min(self._nested_priority(marker_id), 2))

        if not centers:
            return np.array([Config.CAM_CX, Config.CAM_CY], dtype=float)

        centers_np = np.array(centers, dtype=float)
        weights_np = np.array(weights, dtype=float)
        return np.average(centers_np, axis=0, weights=weights_np)

    def _clear_nested_candidate(self):
        self.nested_candidate_id = None
        self.nested_candidate_since = None

    def _nested_candidate_stable(self, candidate_id: int, now: float) -> bool:
        if self.nested_candidate_id != candidate_id:
            self.nested_candidate_id = candidate_id
            self.nested_candidate_since = now
            return False

        if self.nested_candidate_since is None:
            self.nested_candidate_since = now
            return False

        return now - self.nested_candidate_since >= self.nested_switch_stable_sec

    def _accept_nested_id(self, marker_id: int):
        if self.nested_current_id != marker_id:
            name = self.nested_tag_names.get(marker_id, 'nested')
            self.get_logger().info(f'🎯 Nested target switched to ID {marker_id} ({name})')
        self.nested_current_id = marker_id
        self.nested_candidate_id = None
        self.nested_candidate_since = None

    def _accept_marker_detection(self, detection: dict):
        self.marker_tvec = detection['tvec']
        self.marker_rvec = detection['rvec']
        self.marker_center_px = detection['center_px']
        self.marker_id = int(detection['id'])
        self.marker_size = float(detection['size'])
        self.marker_detected = True
        self.last_detection_time = time.time()

    def _clear_marker_if_stale(self):
        # Keep the last detection briefly so single-frame dropouts do not
        # interrupt the precision landing descent.
        if time.time() - self.last_detection_time <= Config.MARKER_LOST_TIMEOUT:
            return

        self.marker_detected = False
        self.marker_tvec = None
        self.marker_rvec = None
        self.marker_center_px = None
        self.marker_id = None

    def _publish_annotated_frame(self, msg: Image, frame: np.ndarray, detections: List[dict],
                                 selected: Optional[dict], rejected) -> None:
        if self.pub_annotated is None and not self.show_debug_view:
            return

        annotated = frame.copy()
        if detections:
            draw_corners = [det['corners'] for det in detections]
            draw_ids = np.array([[int(det['id'])] for det in detections], dtype=np.int32)
            aruco.drawDetectedMarkers(annotated, draw_corners, draw_ids)

        selected_id = None if selected is None else int(selected['id'])
        for detection in detections:
            marker_id = int(detection['id'])
            center = detection['center_px'].astype(int)
            selected_marker = marker_id == selected_id
            color = (0, 255, 0) if selected_marker else (255, 200, 0)
            name = detection['name']
            prefix = 'SELECTED ' if selected_marker else ''
            label = (
                f"{prefix}ID {marker_id} {name} "
                f"z={detection['tvec'][2]:.2f}m size={detection['size']:.2f}m"
            )
            cv2.circle(annotated, tuple(center), 5, color, -1)
            if selected_marker:
                cv2.drawMarker(
                    annotated,
                    tuple(center),
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=34,
                    thickness=2,
                )
            cv2.putText(
                annotated,
                label,
                (int(center[0]) + 8, max(24, int(center[1]) - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        rejected_count = 0 if rejected is None else len(rejected)
        error_text = (
            f" err={self.target_rel_norm:.2f}m"
            if np.isfinite(self.target_rel_norm) else " err=--"
        )
        if selected is not None:
            status = (
                f"Nested={self.nested_mode} detected={self.visible_marker_ids} "
                f"selected={selected_id} {selected['name']} "
                f"state={self.state}{error_text}"
            )
            status_color = (0, 255, 0)
        elif self.visible_marker_ids:
            status = (
                f"Nested={self.nested_mode} detected={self.visible_marker_ids} "
                f"selected=none state={self.state}{error_text}"
            )
            status_color = (0, 220, 255)
        else:
            status = (
                f"Nested={self.nested_mode} detected=none rejected={rejected_count} "
                f"state={self.state}{error_text}"
            )
            status_color = (0, 220, 255)

        cv2.rectangle(annotated, (8, 8), (900, 44), (0, 0, 0), -1)
        cv2.putText(
            annotated,
            status,
            (18, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2,
            cv2.LINE_AA,
        )

        if self.pub_annotated is not None:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.pub_annotated.publish(annotated_msg)

        if self.show_debug_view:
            preview = cv2.resize(annotated, (960, 540))
            cv2.imshow(getattr(self, '_debug_window_title', 'ArUco precision landing'), preview)
            cv2.waitKey(1)

    # ══════════════════════════════════════════════════════════════
    #  CONTROL LOOP
    # ══════════════════════════════════════════════════════════════

    def _loop(self):
        if self.nested_mode and self.use_cpp_tracker:
            self._clear_marker_if_stale()

        offboard_states = {
            'INIT', 'TAKEOFF', 'FLY_TO_SEARCH', 'GIMBAL_DOWN',
            'SEARCH', 'HORIZONTAL_APPROACH', 'DESCEND_OVER_TARGET',
            'FINAL_APPROACH', 'TARGET_LOST',
        }

        if self.state in offboard_states:
            self._pub_offboard()

        # State machine dispatch
        handler = getattr(self, f'_state_{self.state.lower()}', None)
        if handler:
            handler()
        else:
            self.get_logger().error(f'Unknown state: {self.state}')

        if self.state in offboard_states:
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
            self.warmup_count = 0
            self._transition('GIMBAL_DOWN')

    def _state_fly_to_search(self):
        """Fly horizontally to approximate marker location"""
        cfg = Config
        self.sp = np.array([cfg.SEARCH_POS[0], cfg.SEARCH_POS[1], -cfg.CRUISE_ALT])

        dist = np.linalg.norm(self.pos[:2] - np.array(cfg.SEARCH_POS))
        if dist < 0.5:
            target_type = 'Fractal ArUco' if self.use_cpp_tracker else ('ArUco' if not self.nested_mode else 'ArUco')
            self.get_logger().info(f'📍 Arrived at search area — starting {target_type} search')
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
        """Hold position and wait for ArUco detection."""
        cfg = Config
        self.sp = np.array([cfg.SEARCH_POS[0], cfg.SEARCH_POS[1], -cfg.CRUISE_ALT])

        if self._marker_recent():
            self.get_logger().info(
                f'🎯 {self._marker_label()} DETECTED! tvec=[{self.marker_tvec[0]:.2f}, '
                f'{self.marker_tvec[1]:.2f}, {self.marker_tvec[2]:.2f}]')
            self.target_abs = self._pad_target_abs()
            if self.nested_mode:
                self._reset_nested_visual_servo()
                if not self._update_nested_visual_error():
                    self.get_logger().warn('Nested target detected but visual-servo offset is not usable yet')
                    return
            else:
                self._update_target_from_vision()
            self._align_start = time.time()
            self._target_counter = 0
            self._centered_count = 0
            self._transition('HORIZONTAL_APPROACH')
            return

        self._log_search_diagnostics()

        # Timeout check
        elapsed = time.time() - self._search_start
        if elapsed > cfg.SEARCH_TIMEOUT:
            self.get_logger().warn('⏰ Search timeout — landing at current position')
            self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._transition('LAND')

    def _state_horizontal_approach(self):
        """PX4 phase 1: move horizontally over the target while holding altitude."""
        if self.nested_mode:
            self._state_nested_visual_align()
            return

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
        if self.nested_mode:
            self._state_nested_visual_descend()
            return

        cfg = Config

        if not self._marker_recent():
            self._start_target_lost('target lost during descent', search_required=False)
            return

        if not self._update_target_from_vision():
            self._start_target_lost('vision correction unavailable during descent', search_required=False)
            return

        self._send_current_landing_target()

        if self.target_rel_norm > cfg.DESCENT_HACC_RAD:
            self._descent_drift_count += 1
            self.get_logger().warn(
                f'Descent target drift: visual_error={self.target_rel_norm:.2f}m, '
                f'count={self._descent_drift_count}/{cfg.DESCENT_DRIFT_CONFIRM_COUNT}',
                throttle_duration_sec=1.0)
            if self._descent_drift_count >= cfg.DESCENT_DRIFT_CONFIRM_COUNT:
                self.get_logger().warn('Confirmed descent drift — returning to horizontal approach')
                self._target_counter = 0
                self._align_start = time.time()
                self._centered_count = 0
                self._descent_drift_count = 0
                self._transition('HORIZONTAL_APPROACH')
                return
        else:
            self._descent_drift_count = 0

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
            marker_state = self._marker_label() if self._marker_recent() else 'lost'
            self.get_logger().info(
                f'🛬 Final approach: alt={-self.pos[2]:.2f}m, marker={marker_state}')

        self._target_counter += 1

        if self.pos[2] > -(cfg.FINAL_ALT + 0.05):
            self.get_logger().info('🛬 0.1m reached — switching to normal land/disarm')
            self._transition('LAND')

    def _state_target_lost(self):
        """PX4 required-mode search behaviour after PLD_BTOUT target loss."""
        if self.nested_mode:
            self._state_nested_target_lost()
            return

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
        """Let PX4 own touchdown and disarm only after its land detector agrees."""
        cfg = Config
        if self.nested_mode:
            if self._marker_recent() and self._update_nested_visual_error():
                self._send_current_landing_target()
                self.sp = self._nested_visual_setpoint(-cfg.FINAL_ALT, self.nested_max_descent_step)
            else:
                self.sp = np.array([self.pos[0], self.pos[1], -cfg.FINAL_ALT])
        elif self._marker_recent():
            self._update_target_from_vision()
            self._send_current_landing_target()
        elif self.target_abs is None:
            self.target_abs = self._pad_target_abs()

        if not self.nested_mode:
            self.sp = np.array([self.target_abs[0], self.target_abs[1], -cfg.FINAL_ALT])

        if not self._land_cmd_sent:
            self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._land_cmd_sent = True
            self._land_cmd_time = time.time()

        land_elapsed = time.time() - self._land_cmd_time if self._land_cmd_time else 0.0
        near_ground = self.pos[2] > -(cfg.FINAL_ALT + 0.08)

        if not self.armed:
            self.get_logger().info('✅ LANDING COMPLETE!')
            self._transition('DONE')
            return

        if self.landed:
            if self._disarm_retry_counter % cfg.CTRL_HZ == 0:
                self.get_logger().info('🛬 PX4 land detector reports landed — disarming...')
                self._cmd_disarm()
            self._disarm_retry_counter += 1
            return

        now = time.time()
        if now - self._last_land_wait_log > cfg.LAND_WAIT_WARN_INTERVAL:
            self._last_land_wait_log = now
            self.get_logger().warn(
                'Waiting for PX4 land detector before disarm: '
                f'alt={-self.pos[2]:.2f}m, near_ground={near_ground}, '
                f'ground_contact={self.ground_contact}, maybe_landed={self.maybe_landed}, '
                f'at_rest={self.at_rest}, elapsed={land_elapsed:.1f}s')

        if (
            self.allow_force_disarm
            and land_elapsed > cfg.FORCE_DISARM_DELAY
            and near_ground
            and self.ground_contact
            and self.maybe_landed
            and not self._force_disarm_sent
        ):
            self.get_logger().error(
                'allow_force_disarm=true: PX4 still reports not landed, sending force disarm')
            self._cmd_force_disarm()
            self._force_disarm_sent = True

    def _state_done(self):
        """Mission complete"""
        pass

    # ══════════════════════════════════════════════════════════════
    #  MAVLINK LANDING TARGET
    # ══════════════════════════════════════════════════════════════

    def _pad_target_abs(self) -> np.ndarray:
        return np.array([Config.SEARCH_POS[0], Config.SEARCH_POS[1], Config.LANDING_TARGET_Z])

    def _marker_recent(self) -> bool:
        return self.marker_detected and (time.time() - self.last_detection_time) < Config.TARGET_CURRENT_TIMEOUT

    def _marker_label(self) -> str:
        if self.marker_id is None:
            return 'marker'
        name = self.nested_tag_names.get(self.marker_id, 'ArUco')
        return f'ID {self.marker_id} ({name})'

    def _log_search_diagnostics(self):
        now = time.time()
        if now - self._last_search_diag < Config.SEARCH_DIAG_INTERVAL:
            return

        self._last_search_diag = now
        if self.nested_mode and self.use_cpp_tracker:
            if self.camera_frame_count == 0:
                self.get_logger().warn(
                    f'Waiting for tracker poses on {self.pose_topic}. '
                    'The tracker may still print "No fractal marker yet" until '
                    'the UAV reaches the ENU search point and the gimbal is down.',
                    throttle_duration_sec=Config.SEARCH_DIAG_INTERVAL)
                return

            pose_age = now - self.last_camera_frame_time
            self.get_logger().info(
                f'Tracker poses received ({self.camera_frame_count}); '
                f'last pose age={pose_age:.1f}s',
                throttle_duration_sec=Config.SEARCH_DIAG_INTERVAL)
            return

        if self.camera_frame_count == 0:
            self.get_logger().warn(
                f'No {self.camera_topic} frames received. Check the Gazebo camera bridge.',
                throttle_duration_sec=Config.SEARCH_DIAG_INTERVAL)
            return

        frame_age = now - self.last_camera_frame_time
        if frame_age > 1.0:
            self.get_logger().warn(
                f'{self.camera_topic} is stale ({frame_age:.1f}s since last frame). Check bridge/DDS env.',
                throttle_duration_sec=Config.SEARCH_DIAG_INTERVAL)
            return

        self.get_logger().info(
            f'Camera frames received ({self.camera_frame_count}); '
            f'visible ArUco IDs={self.visible_marker_ids or "none"}',
            throttle_duration_sec=Config.SEARCH_DIAG_INTERVAL)

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

    def _reset_nested_visual_servo(self):
        self.nested_visual_rel_raw = None
        self.nested_visual_rel_filtered = None
        self.nested_visual_camera_xy = None
        self.nested_visual_last_update = 0.0
        self.target_rel_xy = None
        self.target_rel_norm = float('inf')
        self.nested_last_good_xy = self.pos[:2].copy()

    def _ned_xy_to_enu_xy(self, ned_xy: np.ndarray) -> np.ndarray:
        return np.array([float(ned_xy[1]), float(ned_xy[0])], dtype=float)

    def _enu_xy_to_ned_xy(self, enu_xy: np.ndarray) -> np.ndarray:
        return np.array([float(enu_xy[1]), float(enu_xy[0])], dtype=float)

    def _ned_sp_to_enu_text(self, ned_sp: Optional[np.ndarray] = None) -> str:
        sp = self.sp if ned_sp is None else ned_sp
        return f'[{sp[1]:.2f},{sp[0]:.2f},{-sp[2]:.2f}]'

    def _rotate_body_enu_to_local_enu(self, body_enu_xy: np.ndarray) -> np.ndarray:
        """Rotate body-carried ENU horizontal vector into local ENU."""
        yaw_ned = self._yaw_from_attitude()
        east_body = float(body_enu_xy[0])
        north_body = float(body_enu_xy[1])
        c = math.cos(yaw_ned)
        s = math.sin(yaw_ned)
        return np.array([
            s * north_body + c * east_body,
            c * north_body - s * east_body,
        ], dtype=float)

    def _camera_xy_to_local_enu(self, camera_xy: np.ndarray) -> np.ndarray:
        """Map signed camera horizontal axes into the local ENU control frame."""
        if self.camera_yaw_frame == 'local':
            return camera_xy.copy()
        return self._rotate_body_enu_to_local_enu(camera_xy)

    def _nested_marker_pose_rel_enu(self) -> Optional[np.ndarray]:
        """Estimate nested marker horizontal offset in local ENU from solvePnP tvec."""
        if self.marker_tvec is None:
            return None

        cfg = Config
        camera_xy = np.array([
            self.camera_x_to_body_east_sign * float(self.marker_tvec[0]),
            self.camera_y_to_body_north_sign * float(self.marker_tvec[1]),
        ], dtype=float)
        rel = self._camera_xy_to_local_enu(camera_xy)
        self.nested_visual_camera_xy = camera_xy

        norm = float(np.linalg.norm(rel))
        if norm > cfg.MAX_VISUAL_CORRECTION:
            self.get_logger().warn(
                f'Nested pose correction rejected: {norm:.2f}m from tracker pose '
                f'(camera_yaw_frame={self.camera_yaw_frame}, '
                f'camera_xy=[{camera_xy[0]:.2f},{camera_xy[1]:.2f}])',
                throttle_duration_sec=1.0)
            return None

        return rel

    def _update_nested_visual_error(self) -> bool:
        rel_enu_xy = self._nested_marker_pose_rel_enu()
        if rel_enu_xy is None:
            return False

        alpha = min(1.0, max(0.0, self.nested_visual_alpha))
        if self.nested_visual_rel_filtered is None:
            filtered_enu = rel_enu_xy
        else:
            filtered_enu = (
                (1.0 - alpha) * self.nested_visual_rel_filtered
                + alpha * rel_enu_xy
            )

        rel_ned_xy = self._enu_xy_to_ned_xy(filtered_enu)
        current_enu_xy = self._ned_xy_to_enu_xy(self.pos[:2])
        target_enu_xy = current_enu_xy + filtered_enu

        self.nested_visual_rel_raw = rel_enu_xy
        self.nested_visual_rel_filtered = filtered_enu
        self.nested_visual_last_update = time.time()
        self.target_rel_xy = rel_ned_xy
        self.target_rel_norm = float(np.linalg.norm(filtered_enu))
        self.target_abs = np.array([
            target_enu_xy[1],
            target_enu_xy[0],
            Config.LANDING_TARGET_Z,
        ])
        return True

    def _nested_visual_setpoint(self, target_z: float, max_step: float) -> np.ndarray:
        rel_enu_xy = self.nested_visual_rel_filtered
        if rel_enu_xy is None:
            return np.array([self.pos[0], self.pos[1], target_z])

        delta_enu_xy = self.nested_servo_gain * rel_enu_xy
        dist = float(np.linalg.norm(delta_enu_xy))
        if dist > max_step:
            delta_enu_xy = delta_enu_xy * (max_step / dist)

        target_enu_xy = self._ned_xy_to_enu_xy(self.pos[:2]) + delta_enu_xy
        target_ned_xy = self._enu_xy_to_ned_xy(target_enu_xy)
        self.nested_last_good_xy = target_ned_xy.copy()
        return np.array([target_ned_xy[0], target_ned_xy[1], target_z])

    def _state_nested_visual_align(self):
        """Nested mode: center over the active/fused tag before descending."""
        cfg = Config

        if not self._marker_recent():
            self._start_target_lost('nested target lost during visual align')
            return

        if not self._update_nested_visual_error():
            self._start_target_lost('nested visual correction unavailable during align')
            return

        self._send_current_landing_target()
        hold_z = float(self.pos[2]) if -float(self.pos[2]) < cfg.CRUISE_ALT - 0.15 else -cfg.CRUISE_ALT
        self.sp = self._nested_visual_setpoint(hold_z, self.nested_max_align_step)

        if self._target_counter % cfg.CTRL_HZ == 0:
            raw = self.nested_visual_rel_raw
            raw_text = 'raw_enu=--' if raw is None else f'raw_enu=[{raw[0]:.2f},{raw[1]:.2f}]'
            cam = self.nested_visual_camera_xy
            cam_text = 'camera_xy=--' if cam is None else f'camera_xy=[{cam[0]:.2f},{cam[1]:.2f}]'
            pos_enu = self._ned_xy_to_enu_xy(self.pos[:2])
            target_enu = (
                self._ned_xy_to_enu_xy(self.target_abs[:2])
                if self.target_abs is not None else np.array([float('nan'), float('nan')])
            )
            self.get_logger().info(
                f'🧭 Nested visual align: id={self.marker_id}, '
                f'err={self.target_rel_norm:.2f}m, {cam_text}, {raw_text}, '
                f'pos_enu=[{pos_enu[0]:.2f},{pos_enu[1]:.2f}], '
                f'target_enu=[{target_enu[0]:.2f},{target_enu[1]:.2f}], '
                f'centered={self._centered_count}/{self.nested_align_confirm_count}, '
                f'sp_enu={self._ned_sp_to_enu_text()}, '
                f'yaw_ned={self._yaw_from_attitude():.2f}, frame={self.camera_yaw_frame}')

        self._target_counter += 1

        if self.target_rel_norm < self.nested_align_hacc_rad:
            self._centered_count += 1
        else:
            self._centered_count = 0

        if self._centered_count >= self.nested_align_confirm_count:
            self.get_logger().info('✅ Nested target centered — starting visual descent')
            self._target_counter = 0
            self._centered_count = 0
            self._descent_drift_count = 0
            self._descent_z_sp = float(self.pos[2])
            self._transition('DESCEND_OVER_TARGET')
            return

        if time.time() - self._align_start > cfg.ALIGN_TIMEOUT:
            self.get_logger().warn(
                'Nested visual align still not centered; keeping visual servo active '
                f'instead of timing out. err={self.target_rel_norm:.2f}m',
                throttle_duration_sec=2.0)

    def _state_nested_visual_descend(self):
        """Nested mode: descend only while visual error remains acceptable."""
        cfg = Config

        if not self._marker_recent():
            self._start_target_lost('nested target lost during visual descent', search_required=False)
            return

        if not self._update_nested_visual_error():
            self._start_target_lost('nested visual correction unavailable during descent', search_required=False)
            return

        self._send_current_landing_target()

        descent_allowed = self.target_rel_norm <= self.nested_descent_hacc_rad
        if descent_allowed:
            self._descent_drift_count = 0
            target_final_z = -self.nested_final_alt
            self._descent_z_sp = min(
                target_final_z,
                self._descent_z_sp + cfg.DESCENT_RATE / cfg.CTRL_HZ
            )
        else:
            self._descent_drift_count += 1
            self._descent_z_sp = float(self.pos[2])
            self.get_logger().warn(
                f'Nested descent paused for realign: err={self.target_rel_norm:.2f}m '
                f'> {self.nested_descent_hacc_rad:.2f}m, '
                f'count={self._descent_drift_count}',
                throttle_duration_sec=1.0)

        self.sp = self._nested_visual_setpoint(self._descent_z_sp, self.nested_max_descent_step)

        if self._target_counter % cfg.CTRL_HZ == 0:
            phase = 'descending' if descent_allowed else 'align-hold'
            raw = self.nested_visual_rel_raw
            raw_text = 'raw_enu=--' if raw is None else f'raw_enu=[{raw[0]:.2f},{raw[1]:.2f}]'
            cam = self.nested_visual_camera_xy
            cam_text = 'camera_xy=--' if cam is None else f'camera_xy=[{cam[0]:.2f},{cam[1]:.2f}]'
            pos_enu = self._ned_xy_to_enu_xy(self.pos[:2])
            target_enu = (
                self._ned_xy_to_enu_xy(self.target_abs[:2])
                if self.target_abs is not None else np.array([float('nan'), float('nan')])
            )
            self.get_logger().info(
                f'🛬 Nested visual descent ({phase}): id={self.marker_id}, '
                f'alt={-self.pos[2]:.2f}m, target_alt={-self._descent_z_sp:.2f}m, '
                f'err={self.target_rel_norm:.2f}m, {cam_text}, {raw_text}, '
                f'pos_enu=[{pos_enu[0]:.2f},{pos_enu[1]:.2f}], '
                f'target_enu=[{target_enu[0]:.2f},{target_enu[1]:.2f}], '
                f'sp_enu={self._ned_sp_to_enu_text()}, '
                f'frame={self.camera_yaw_frame}')

        self._target_counter += 1

        if -self.pos[2] <= self.nested_final_alt + 0.05:
            self.get_logger().info(
                f'🛬 Fractal final altitude reached ({self.nested_final_alt:.2f}m) — switching to PX4 land')
            self._target_counter = 0
            self._transition('LAND')

    def _state_nested_target_lost(self):
        """Nested mode: hold briefly, then climb back to search instead of guessing touchdown."""
        cfg = Config
        if self._target_lost_start is None:
            self._target_lost_start = time.time()

        if self._marker_recent() and self._update_nested_visual_error():
            resume = 'DESCEND_OVER_TARGET' if self._target_lost_from_state == 'DESCEND_OVER_TARGET' else 'HORIZONTAL_APPROACH'
            self.get_logger().info(f'🎯 Nested target reacquired — resuming {resume}')
            self._target_counter = 0
            self._centered_count = 0
            self._target_lost_start = None
            self._target_lost_from_state = None
            if resume == 'HORIZONTAL_APPROACH':
                self._align_start = time.time()
            self._transition(resume)
            return

        elapsed = time.time() - self._target_lost_start
        if elapsed < self.nested_target_loss_grace:
            self.sp = np.array([
                self.nested_last_good_xy[0],
                self.nested_last_good_xy[1],
                float(self.pos[2]),
            ])
            if self._target_counter % cfg.CTRL_HZ == 0:
                self.get_logger().warn(
                    f'Nested target temporarily lost — holding last visual setpoint, '
                    f'elapsed={elapsed:.1f}s/{self.nested_target_loss_grace:.1f}s')
            self._target_counter += 1
            return

        self.sp = np.array([cfg.SEARCH_POS[0], cfg.SEARCH_POS[1], -cfg.CRUISE_ALT])
        dist = float(np.linalg.norm(self.pos[:2] - np.array(cfg.SEARCH_POS)))
        alt_err = abs(self.pos[2] - (-cfg.CRUISE_ALT))
        if self._target_counter % cfg.CTRL_HZ == 0:
            self.get_logger().warn(
                f'Nested target lost — returning to search pose: dist={dist:.2f}m, '
                f'alt_err={alt_err:.2f}m')
        self._target_counter += 1

        if dist < 0.5 and alt_err < 0.35:
            self._search_start = time.time()
            self._target_counter = 0
            self._target_lost_start = None
            self._target_lost_from_state = None
            self._reset_nested_visual_servo()
            self._transition('SEARCH')

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

        # Image x is camera-right; image y is camera-down/south for a nadir view.
        # Rotate the vehicle-carried offset into local NED.
        north_vehicle = -dy_px * meters_per_px_y
        east_vehicle = dx_px * meters_per_px_x
        yaw = self._yaw_from_attitude()
        c = math.cos(yaw)
        s = math.sin(yaw)
        rel = np.array([
            c * north_vehicle - s * east_vehicle,
            s * north_vehicle + c * east_vehicle,
        ], dtype=float)

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
        if self.last_detection_time <= self._last_processed_detection_time:
            if (
                self.target_abs is not None
                and (time.time() - self.last_detection_time) < cfg.VISION_REUSE_TIMEOUT
            ):
                rel_xy = self.target_abs[:2] - self.pos[:2]
                self.target_rel_xy = rel_xy
                self.target_rel_norm = float(np.linalg.norm(rel_xy))
                return True
            return False

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
        self._last_processed_detection_time = self.last_detection_time
        return True

    def _yaw_from_attitude(self) -> float:
        w, x, y, z = [float(v) for v in self.q_att]
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

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
            marker_size = float(self.marker_size if self.marker_size else Config.MARKER_SIZE)
            self.mav_conn.mav.landing_target_send(
                int(time.time() * 1e6),
                0,
                1,  # MAV_FRAME_LOCAL_NED
                0.0,
                0.0,
                float(np.sqrt(rel_x**2 + rel_y**2 + rel_z**2)),
                marker_size,
                marker_size,
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
        if new_state == 'LAND':
            self._disarm_retry_counter = 0
            self._last_land_wait_log = 0.0
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


def fractal_main(args=None):
    rclpy.init(args=args)
    node = ArucoPrecisionLander(
        node_name='fractal_aruco_precision_lander',
        nested_mode_default=True,
    )
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
