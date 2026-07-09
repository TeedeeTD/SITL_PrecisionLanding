#!/usr/bin/env python3
"""Offboard Precision Landing Controller.

Monitors drone flight state via MAVROS. When AUTO.LAND is detected,
takes over in OFFBOARD mode and performs smooth visual precision landing
using the PX4 precland state machine design with Visual Servo control.
"""

from __future__ import annotations
import math
from collections import deque
from typing import Deque, Optional
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from dib_msgs.msg import LandingTarget6D
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State, ExtendedState, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamGet, WaypointPull


class OffboardPreclandController(Node):
    # --- Configuration ---
    CTRL_HZ = 30
    TARGET_TIMEOUT = 1.2
    TRACKING_CONFIRM = 10
    ALIGN_CONFIRM = 6
    ALIGN_TIMEOUT = 18.0
    SEARCH_TIMEOUT = 35.0
    MAX_SEARCH = 3
    DESCENT_RATE = 0.45
    FINAL_ALT = 0.30
    FAPPR_ALT = 0.5
    HACC_RAD = 0.20
    MAX_ALIGN_STEP = 0.45
    MAX_DESCENT_STEP = 0.35
    SERVO_GAIN_HIGH = 0.35
    SERVO_GAIN_LOW = 0.75
    HIGH_ALT = 8.0
    LOW_ALT = 5.0
    ALPHA_HIGH = 0.18
    ALPHA_LOW = 0.45
    FILTER_WINDOW = 7
    ALIGN_R_MIN = 1.2
    ALIGN_R_MAX = 4.0
    ALIGN_R_GAIN = 0.20
    ALIGN_R_BIAS = 0.15
    DESC_R_MIN = 1.5
    DESC_R_MAX = 5.0
    DESC_R_GAIN = 0.25
    DESC_R_BIAS = 0.20
    REJECT_R_MIN = 1.0
    REJECT_R_MAX = 5.0
    REJECT_R_GAIN = 0.50
    TARGET_LOSS_GRACE = 1.5
    DESCENT_LOSS_HOLD = 3.0
    LOW_ALT_COMMIT = 1.0
    LOW_ALT_MAX_ERR = 0.25
    SEARCH_ALT = 10.0          # Độ cao tìm/bắt tag (tag chỉ detect được ≤ ~11m)
    SEARCH_ALT_MAX = 11.0      # Cap cứng — không bao giờ leo cao hơn mức này khi search
    YAW_SLEW_RATE = 0.6        # rad/s — slew heading nhẹ sau khi đã khóa tag
    YAW_LOCK_SAMPLES = 15      # số mẫu yaw gom TẠI YAW_LOCK_ALT trước khi đóng băng heading
    YAW_LOCK_ALT = 7.0         # độ cao (m) dừng lại lấy mẫu yaw rồi đóng băng

    def __init__(self) -> None:
        super().__init__("offboard_precland_controller")

        self.declare_parameter("camera_x_to_body_east_sign", 1.0)
        self.declare_parameter("camera_y_to_body_north_sign", -1.0)
        self.declare_parameter("camera_yaw_frame", "body")
        self.declare_parameter("camera_offset_x", 0.1517)
        self.declare_parameter("camera_offset_y", 0.0)
        self.declare_parameter("marker_size", 0.50)
        self.declare_parameter("target_topic", "/landing/target_camera")
        self.declare_parameter("target_pose_topic", "/aruco_fractal_tracker/poses")
        self.declare_parameter("align_yaw_to_tag", False)  # TẮT mặc định — ArUco solvePnP yaw không tin cậy
        self.declare_parameter("tag_yaw_sign", 1.0)        # lật dấu nếu xoay ngược chiều
        self.declare_parameter("tag_yaw_offset", 0.0)      # bù offset khung camera→body (rad), calib 1 lần
        self.declare_parameter("precland_mode", 2)          # 0=disabled, 1=opportunistic, 2=required
        self.declare_parameter("final_alt", 0.18)           # Độ cao bàn giao quyền lại cho PX4 (m)

        self.cam_east_sign = self.get_parameter("camera_x_to_body_east_sign").value
        self.cam_north_sign = self.get_parameter("camera_y_to_body_north_sign").value
        self.cam_yaw_frame = self.get_parameter("camera_yaw_frame").value.strip().lower()
        self.cam_off_x = self.get_parameter("camera_offset_x").value
        self.cam_off_y = self.get_parameter("camera_offset_y").value
        self.marker_size = self.get_parameter("marker_size").value
        target_topic = self.get_parameter("target_topic").value
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.align_yaw_to_tag = bool(self.get_parameter("align_yaw_to_tag").value)
        self.tag_yaw_sign = float(self.get_parameter("tag_yaw_sign").value)
        self.tag_yaw_offset = float(self.get_parameter("tag_yaw_offset").value)
        self.precland_mode = int(self.get_parameter("precland_mode").value)
        self.FINAL_ALT = float(self.get_parameter("final_alt").value)

        # State
        self.state = "IDLE"
        self.pos_enu = np.zeros(3)
        self.q_att = np.array([1.0, 0.0, 0.0, 0.0])
        self.sp_enu = np.zeros(3)
        self.sp_yaw = 0.0
        self._held_yaw = 0.0                    # heading giữ nguyên lúc takeover
        self._tag_yaw_abs: Optional[float] = None  # heading ĐÍCH tuyệt đối (ENU) từ tag
        self._yaw_locked = False                # đã chốt & đóng băng heading đích chưa
        self._yaw_lock_buf: list = []           # bộ đệm mẫu để gộp trước khi chốt
        self._yaw_realign_complete = False     # đã hoàn thành xoay yaw và căn chỉnh lại XY tại 7m chưa
        self._realign_cnt = 0                   # đếm mẫu căn chỉnh ổn định sau khi xoay yaw
        self.current_mode = ""
        self.landed_state = 0
        self.is_landing = False
        self.armed = False
        self.mavros_connected = False

        # Target tracking
        self.target_samples: Deque[np.ndarray] = deque(maxlen=self.FILTER_WINDOW)
        self.filtered_rel_enu: Optional[np.ndarray] = None
        self.target_enu: Optional[np.ndarray] = None
        self.target_rel_norm = float("inf")
        self.last_pose_time = 0.0
        self.tracking_count = 0
        self.history = deque(maxlen=150)
        self._waypoints = []
        self._current_wp_seq = 0

        # FSM counters
        self._centered_count = 0
        self._descent_drift_count = 0
        self._descent_z_sp = 10.0
        self._target_counter = 0
        self._search_start: Optional[float] = None
        self._align_start: Optional[float] = None
        self._search_cnt = 0
        self._approach_alt = 10.0
        self._start_z_sp = 10.0                 # ramp độ cao khi hạ về vùng detect
        self._target_lost_start: Optional[float] = None
        self._target_lost_from: Optional[str] = None
        self._land_hold_pos: Optional[np.ndarray] = None
        self._gimbal_configured = False
        self._offboard_activated = False

        # QoS
        pose_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        # Publishers
        self.pub_sp = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.pub_state = self.create_publisher(String, "/lander/state", 10)

        # Subscribers
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_pos, pose_qos)
        self.create_subscription(PoseStamped, self.target_pose_topic, self._on_target, 10)
        self.create_subscription(State, "/mavros/state", self._on_state, state_qos)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_ext_state, state_qos)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self._on_waypoints, state_qos)

        # Services
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cmd_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.param_get_client = self.create_client(ParamGet, "/mavros/param/get")
        self.wp_pull_client = self.create_client(WaypointPull, "/mavros/mission/pull")

        # Main loop
        self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.create_timer(2.0, self._gimbal_tick)
        self.create_timer(3.0, self._query_px4_params)

        self.get_logger().info("OffboardPreclandController ready — monitoring for AUTO.LAND")

    # ── Callbacks ──────────────────────────────────────────────

    def _on_pos(self, msg: PoseStamped) -> None:
        self.pos_enu[:] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        q = msg.pose.orientation
        self.q_att[:] = [q.w, q.x, q.y, q.z]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.history.append((stamp, self.pos_enu.copy(), self.q_att.copy()))

    def _on_state(self, msg: State) -> None:
        self.current_mode = msg.mode
        self.armed = msg.armed
        was_connected = self.mavros_connected
        self.mavros_connected = msg.connected
        if msg.connected and not was_connected:
            self.get_logger().info("MAVROS connected — pulling waypoints...")
            self._pull_waypoints_immediately()

        was = self.is_landing
        self.is_landing = (msg.mode == "AUTO.LAND")
        if self.is_landing != was:
            self.get_logger().info(f"Landing flag: {self.is_landing} (mode={msg.mode})")
            if self.is_landing:
                self._pull_waypoints_immediately()

    def _on_ext_state(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state
        was = self.is_landing
        if not self.is_landing and msg.landed_state == ExtendedState.LANDED_STATE_LANDING:
            self.is_landing = True
        if self.is_landing != was:
            self.get_logger().info(f"Landing flag: {self.is_landing} (landed_state={msg.landed_state})")
            if self.is_landing:
                self._pull_waypoints_immediately()

    def _on_waypoints(self, msg: WaypointList) -> None:
        self._waypoints = msg.waypoints
        self._current_wp_seq = msg.current_seq
        self.get_logger().info(f"Received waypoints update: {len(msg.waypoints)} items. Active seq: {msg.current_seq}")

    def _pull_waypoints_immediately(self):
        if self.wp_pull_client.service_is_ready():
            req_wp = WaypointPull.Request()
            self.wp_pull_client.call_async(req_wp)
            self.get_logger().info("Landing phase started — pulling waypoints immediately")

    def _on_target(self, msg: PoseStamped) -> None:
        self.tracking_count += 1
        tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        cam_xy = np.array([self.cam_east_sign * tvec[0], self.cam_north_sign * tvec[1]])

        t_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        h_pos, h_q = self._hist_state(t_time)
        rel = self._cam_to_enu(cam_xy, h_q)

        rn = float(np.linalg.norm(rel))
        rr = self._reject_r()
        if rn > rr:
            self.tracking_count = 0
            return

        abs_xy = h_pos[:2] + rel
        self.target_enu = abs_xy.copy()
        self.target_rel_norm = float(np.linalg.norm(self.target_enu - self.pos_enu[:2]))
        self.last_pose_time = self._now()

        # Heading ĐÍCH tuyệt đối (ENU) bằng phép nhân quaternion 3D trực tiếp
        if self.align_yaw_to_tag:
            # Trích xuất quaternion của tag trong hệ tọa độ camera
            q_tag_cam = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ])
            # Camera to body (optical to FLU)
            # R_cam_body tương ứng với q_cam_body = [0.0, 0.7071067811865475, -0.7071067811865475, 0.0]
            q_cam_body = np.array([0.0, 0.7071067811865475, -0.7071067811865475, 0.0])

            # Hướng tuyệt đối của tag trong hệ thế giới (ENU):
            # q_tag_world = h_q * q_cam_body * q_tag_cam
            q_tag_world = self._q_mul(self._q_mul(h_q, q_cam_body), q_tag_cam)

            # Trích xuất góc yaw tuyệt đối trong hệ thế giới (ENU)
            world_yaw_sample = self._yaw(q_tag_world)

            # Áp dụng bù tag_yaw_offset nếu có
            if self.tag_yaw_offset != 0.0:
                world_yaw_sample = self._wrap(world_yaw_sample + self.tag_yaw_offset)

            # PHÂN PHA THEO ĐỘ CAO: chỉ gom mẫu khi ĐÃ HẠ tới YAW_LOCK_ALT (7m) trong
            # pha DESCEND và CHƯA chốt. Đủ YAW_LOCK_SAMPLES mẫu → đóng băng heading NGAY.
            # Trên 7m (APPROACH + DESCEND 10→7): KHÔNG gom, KHÔNG xoay yaw (giữ heading).
            if (self.state == "DESCEND_ABOVE_TARGET" and not self._yaw_locked
                    and self.pos_enu[2] <= self.YAW_LOCK_ALT):
                self._yaw_lock_buf.append(world_yaw_sample)
                if len(self._yaw_lock_buf) >= self.YAW_LOCK_SAMPLES:
                    self._latch_yaw()

            if self.tracking_count % 15 == 0:
                body_yaw = self._yaw(h_q)
                tgt = (f"{math.degrees(self._tag_yaw_abs):.1f}°"
                       if self._tag_yaw_abs is not None else "—")
                self.get_logger().info(
                    f"[YAW-3D] alt={self.pos_enu[2]:.1f}m body={math.degrees(body_yaw):.1f}° | "
                    f"world_sample={math.degrees(world_yaw_sample):.1f}° | "
                    f"locked={self._yaw_locked} target={tgt} | "
                    f"buf={len(self._yaw_lock_buf)}/{self.YAW_LOCK_SAMPLES} | "
                    f"sp={math.degrees(self.sp_yaw):.1f}°"
                )

    # ── Time-sync helpers ─────────────────────────────────────

    def _hist_state(self, t: float):
        if not self.history:
            return self.pos_enu.copy(), self.q_att.copy()
        best = (self.history[-1][1], self.history[-1][2])
        bd = float("inf")
        for ts, p, q in self.history:
            d = abs(ts - t)
            if d < bd:
                bd = d
                best = (p, q)
        return best[0].copy(), best[1].copy()

    def _yaw(self, q):
        w, x, y, z = [float(v) for v in q]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _q_mul(q1, q2):
        w1, x1, y1, z1 = [float(v) for v in q1]
        w2, x2, y2, z2 = [float(v) for v in q2]
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _circmean(angles):
        s = sum(math.sin(a) for a in angles)
        c = sum(math.cos(a) for a in angles)
        return math.atan2(s, c)

    def _latch_yaw(self):
        """Chốt & ĐÓNG BĂNG heading đích từ các mẫu gom TẠI YAW_LOCK_ALT (7m).
        Gọi khi bộ đệm đã đủ YAW_LOCK_SAMPLES mẫu."""
        if not self.align_yaw_to_tag or self._yaw_locked:
            return
        if self._yaw_lock_buf:
            self._tag_yaw_abs = self._circmean(self._yaw_lock_buf)
            self._yaw_locked = True
            self.get_logger().info(
                f"[YAW-LOCK] chốt target={math.degrees(self._tag_yaw_abs):.1f}° "
                f"từ {len(self._yaw_lock_buf)} mẫu tại {self.pos_enu[2]:.1f}m — "
                f"đóng băng, tiếp tục căn + hạ với yaw đã căn"
            )
        else:
            self.get_logger().warn(
                "[YAW-LOCK] chưa gom được mẫu yaw nào — giữ heading"
            )

    def _desired_yaw(self):
        """Trước khi CHỐT (APPROACH + DESCEND trên 7m): GIỮ heading (_held_yaw) —
        KHÔNG xoay yaw, đúng như align_yaw_to_tag=False. Chốt chỉ xảy ra tại 7m sau khi
        gom đủ mẫu. Sau khi chốt: luôn lệnh heading ĐÓNG BĂNG (hằng số)."""
        if not (self.align_yaw_to_tag and self._yaw_locked
                and self._tag_yaw_abs is not None):
            return self._held_yaw
        return self._tag_yaw_abs

    def _update_yaw(self):
        """Slew sp_yaw về heading mong muốn — tránh giật yaw."""
        desired = self._desired_yaw()
        step = self.YAW_SLEW_RATE / self.CTRL_HZ
        err = self._wrap(desired - self.sp_yaw)
        if abs(err) <= step:
            self.sp_yaw = desired
        else:
            self.sp_yaw = self._wrap(self.sp_yaw + math.copysign(step, err))

    def _cam_to_enu(self, cam_xy, q_att):
        if self.cam_yaw_frame == "local":
            return cam_xy.copy()
        yaw = self._yaw(q_att)
        eb = float(cam_xy[0])
        nb = float(cam_xy[1])
        xb = nb + self.cam_off_x
        yb = -eb + self.cam_off_y
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([xb * c - yb * s, xb * s + yb * c])

    # ── Utility ───────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _alt(self):
        return max(0.0, float(self.pos_enu[2]))

    def _blend(self):
        span = max(0.1, self.HIGH_ALT - self.LOW_ALT)
        return min(1.0, max(0.0, (self._alt() - self.LOW_ALT) / span))

    def _alpha(self):
        t = self._blend()
        return self.ALPHA_LOW * (1.0 - t) + self.ALPHA_HIGH * t

    def _servo_gain(self):
        t = self._blend()
        return self.SERVO_GAIN_LOW * (1.0 - t) + self.SERVO_GAIN_HIGH * t

    def _align_r(self):
        v = self.ALIGN_R_BIAS + self.ALIGN_R_GAIN * self._alt()
        return min(self.ALIGN_R_MAX, max(self.ALIGN_R_MIN, v))

    def _descent_r(self):
        v = self.DESC_R_BIAS + self.DESC_R_GAIN * self._alt()
        return min(self.DESC_R_MAX, max(self.DESC_R_MIN, v))

    def _reject_r(self):
        v = self.REJECT_R_GAIN * self._alt()
        return min(self.REJECT_R_MAX, max(self.REJECT_R_MIN, v))

    def _target_fresh(self):
        return (self._now() - self.last_pose_time) < self.TARGET_TIMEOUT and self.last_pose_time > 0

    def _visual_sp(self, z, max_step):
        if self.target_enu is None:
            return np.array([self.pos_enu[0], self.pos_enu[1], z])
        rel = self.target_enu - self.pos_enu[:2]
        delta = self._servo_gain() * rel
        d = float(np.linalg.norm(delta))
        if d > max_step:
            delta *= max_step / d
        xy = self.pos_enu[:2] + delta
        return np.array([xy[0], xy[1], z])

    def _transition(self, new_state):
        old = self.state
        self.state = new_state
        self.get_logger().info(f"FSM: {old} → {new_state}")

        if new_state in ("IDLE", "START", "HORIZONTAL_APPROACH"):
            self._yaw_locked = False
            self._tag_yaw_abs = None
            self._yaw_lock_buf.clear()
            self._yaw_realign_complete = False
            self._realign_cnt = 0

    # ── Mode switching ────────────────────────────────────────

    def _set_mode(self, mode: str):
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def _query_px4_params(self):
        """Query PX4 parameter RTL_PLD_MD to automatically sync the precision landing mode."""
        if self.param_get_client.service_is_ready():
            req = ParamGet.Request()
            req.param_id = "RTL_PLD_MD"
            future = self.param_get_client.call_async(req)
            future.add_done_callback(self._on_param_received)

    def _on_param_received(self, future):
        try:
            res = future.result()
            if res.success:
                val = int(res.value.integer)
                if val in (0, 1, 2):
                    if self.precland_mode != val:
                        self.get_logger().info(f"Automatically synced PX4 RTL_PLD_MD param: {self.precland_mode} -> {val}")
                        self.precland_mode = val
        except Exception as exc:
            self.get_logger().warn(f"Failed to query PX4 parameter RTL_PLD_MD: {exc}", throttle_duration_sec=10.0)

    def _cmd(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        if not self.cmd_client.service_is_ready():
            return
        req = CommandLong.Request()
        req.command = command
        req.param1, req.param2, req.param3 = p1, p2, p3
        req.param4, req.param5, req.param6, req.param7 = p4, p5, p6, p7
        self.cmd_client.call_async(req)

    def _disarm(self):
        if not self.arm_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = False
        self.arm_client.call_async(req)

    # ── Gimbal ────────────────────────────────────────────────

    def _gimbal_tick(self):
        if not self.cmd_client.service_is_ready():
            return
        if not self._gimbal_configured:
            self._cmd(1001, p1=1.0, p2=191.0)
            self._gimbal_configured = True
        pitch = -90.0 if self.state not in ("IDLE", "DONE") else 0.0
        self._cmd(1000, p1=pitch, p2=0.0, p3=float("nan"), p4=float("nan"), p5=0.0)
        self._cmd(205, p1=pitch, p2=0.0, p3=0.0, p7=2.0)

    # ── Setpoint publishing ───────────────────────────────────

    def _pub_setpoint(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self.sp_enu[0])
        msg.pose.position.y = float(self.sp_enu[1])
        msg.pose.position.z = float(self.sp_enu[2])
        msg.pose.orientation.z = math.sin(self.sp_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.sp_yaw / 2.0)
        self.pub_sp.publish(msg)

    # ── Main loop ─────────────────────────────────────────────

    def _loop(self):
        # Nếu mất tag (dữ liệu pose bị quá hạn), dọn dẹp các bộ đệm target
        if not self._target_fresh():
            self.target_samples.clear()
            self.filtered_rel_enu = None
            self.tracking_count = 0

        handler = getattr(self, f"_st_{self.state.lower()}", None)
        if handler:
            handler()

        self._update_yaw()

        if self.state not in ("IDLE", "DONE", "FALLBACK", "FINAL_APPROACH"):
            self._pub_setpoint()

        try:
            m = String()
            m.data = self.state
            self.pub_state.publish(m)
        except Exception:
            pass

    # ── FSM States ────────────────────────────────────────────

    def _st_idle(self):
        """Monitor for AUTO.LAND — chỉ can thiệp khi precland_mode > 0."""
        if not (self.is_landing and self.armed):
            return

        # Tự động phát hiện chế độ precision landing cho điểm hạ cánh hiện tại trong bài bay (mission)
        active_mode = self.precland_mode
        land_wp = None
        # Kiểm tra cả waypoint hiện tại và waypoint tiếp theo (tránh trễ đồng bộ active seq từ PX4)
        for idx in (self._current_wp_seq, self._current_wp_seq + 1):
            if self._waypoints and idx < len(self._waypoints):
                wp = self._waypoints[idx]
                if wp.command in (21, 85):
                    land_wp = wp
                    break

        if land_wp is not None:
            active_mode = int(land_wp.param2)
            self.get_logger().info(
                f"Mission landing detected: command={land_wp.command}, precision land mode={active_mode} (seq={self._current_wp_seq})"
            )

        if active_mode == 0:
            return  # Precision landing disabled cho lần hạ cánh này — KHÔNG can thiệp

        # Ghi nhận chế độ hoạt động cho lượt hạ cánh này
        self.precland_mode = active_mode

        # Cả mode 1 (opportunistic) và mode 2 (required) đều tự động chiếm quyền OFFBOARD khi bắt đầu land.
        # Sự khác biệt sẽ nằm ở việc xử lý khi không tìm thấy tag ở độ cao detect (10m).
        self.get_logger().info("AUTO.LAND detected — taking over with OFFBOARD precision landing")
        self._land_hold_pos = self.pos_enu.copy()
        self.sp_enu = self.pos_enu.copy()
        self._held_yaw = self._yaw(self.q_att)   # giữ heading, KHÔNG snap về hướng Đông
        self.sp_yaw = self._held_yaw
        self._tag_yaw_abs = None
        self._yaw_locked = False
        self._yaw_lock_buf = []
        self._start_z_sp = float(self.pos_enu[2])
        self._approach_alt = float(self.pos_enu[2])
        self._search_cnt = 0
        self._offboard_activated = False
        self.target_samples.clear()
        self.filtered_rel_enu = None
        self.target_enu = None
        self.target_rel_norm = float("inf")
        self.tracking_count = 0
        self._transition("START")

    def _st_start(self):
        """Activate OFFBOARD mode and wait for target acquisition."""
        # Giữ XY + heading, đồng thời HẠ DẦN về độ cao detect (≤ ~11m) để tag lọt
        # vào tầm nhìn — thay vì hover ở độ cao mission nơi không thể thấy tag.
        hold = self._land_hold_pos if self._land_hold_pos is not None else self.pos_enu
        target_z = min(float(hold[2]), self.SEARCH_ALT)
        self._start_z_sp = max(target_z, self._start_z_sp - self.DESCENT_RATE / self.CTRL_HZ)
        self.sp_enu = np.array([hold[0], hold[1], self._start_z_sp])

        if not self._offboard_activated:
            self._set_mode("OFFBOARD")
            self._offboard_activated = True
            self._search_start = None  # Chỉ bắt đầu tính giờ khi đã hạ độ cao xuống gần SEARCH_ALT
            self.get_logger().info("Requested OFFBOARD mode")
            return

        if self.current_mode != "OFFBOARD":
            if self._target_counter % self.CTRL_HZ == 0:
                self._set_mode("OFFBOARD")
            self._target_counter += 1
            return

        # Target visible? → approach
        if self._target_fresh() and self.tracking_count >= self.TRACKING_CONFIRM:
            self._approach_alt = float(self.pos_enu[2])   # neo ở độ cao detect, không leo lại
            self._align_start = self._now()
            self._target_counter = 0
            self._centered_count = 0
            self._transition("HORIZONTAL_APPROACH")
            return

        # Tính toán timeout tìm kiếm khi đã hạ xuống gần độ cao tìm kiếm (≤ 10.3m)
        if self.pos_enu[2] <= self.SEARCH_ALT + 0.3:
            if self._search_start is None:
                self._search_start = self._now()
                self.get_logger().info(f"Reached search altitude ({self.SEARCH_ALT:.1f}m). Waiting 5s for target acquisition...")

            elapsed = self._now() - self._search_start
            if elapsed > 5.0:
                if self.precland_mode == 1:
                    self.get_logger().warn("Opportunistic mode: Target not found at search altitude → FALLBACK (normal landing)")
                    self._transition("FALLBACK")
                else:
                    self.get_logger().warn("Required mode: Target not found at search altitude → active SEARCH")
                    self._search_start = self._now()
                    self._transition("SEARCH")
        else:
            self._search_start = None

    def _st_horizontal_approach(self):
        """Căn ngang trên target, GIỮ độ cao (~10m), GIỮ heading (chưa xoay yaw)."""
        if not self._target_fresh():
            self.get_logger().warn("Target lost during approach")
            self._target_lost_start = self._now()
            self._target_lost_from = "HORIZONTAL_APPROACH"
            self._transition("TARGET_LOST")
            return

        self.sp_enu = self._visual_sp(self._approach_alt, self.MAX_ALIGN_STEP)
        ar = self._align_r()
        if self.target_rel_norm <= ar:
            self._centered_count += 1
        else:
            self._centered_count = 0

        if self._target_counter % self.CTRL_HZ == 0:
            self.get_logger().info(
                f"APPROACH: alt={self._alt():.1f} err={self.target_rel_norm:.2f} "
                f"gate={ar:.2f} cnt={self._centered_count}/{self.ALIGN_CONFIRM}"
            )
        self._target_counter += 1

        # Ổn định (căn tâm) → bắt đầu hạ. Yaw VẪN chưa xoay; sẽ chốt tại 7m trong DESCEND.
        if self._centered_count >= self.ALIGN_CONFIRM:
            self._descent_z_sp = float(self.pos_enu[2])
            self._target_counter = 0
            self._centered_count = 0
            self._descent_drift_count = 0
            self._transition("DESCEND_ABOVE_TARGET")
            return

        if self._align_start and (self._now() - self._align_start) > self.ALIGN_TIMEOUT:
            dr = self._descent_r()
            if self.target_rel_norm <= dr:
                self._descent_z_sp = float(self.pos_enu[2])
                self._target_counter = 0
                self._transition("DESCEND_ABOVE_TARGET")
                return
            self._align_start = self._now()

    def _st_descend_above_target(self):
        """Vừa căn tâm vừa hạ. Dừng tại YAW_LOCK_ALT (7m) để gom mẫu yaw + đóng băng,
        sau đó tiếp tục căn + hạ với yaw đã căn. Z-lock khi lệch tâm."""
        if not self._target_fresh():
            self._target_lost_start = self._now()
            self._target_lost_from = "DESCEND_ABOVE_TARGET"
            self._transition("TARGET_LOST")
            return

        dr = self._descent_r()
        descent_ok = self.target_rel_norm <= dr

        # Low-altitude commit check
        if self.pos_enu[2] < self.LOW_ALT_COMMIT and not descent_ok:
            age = self._now() - self.last_pose_time
            if age <= 0.5 and self.target_rel_norm <= self.LOW_ALT_MAX_ERR:
                self.get_logger().warn("Low-alt guarded commit → FINAL_APPROACH")
                self._transition("FINAL_APPROACH")
                return

        # Check if we are in the 7m hover phase for yaw alignment & re-centering
        in_7m_hover = (self.align_yaw_to_tag 
                       and not self._yaw_realign_complete 
                       and self.pos_enu[2] <= self.YAW_LOCK_ALT)

        # Căn chỉnh XY mọi lúc, trừ lúc đang xoay Yaw (chênh lệch yaw > 3 độ)
        current_yaw = self._yaw(self.q_att)
        yaw_err = abs(self._wrap(self.sp_yaw - current_yaw))
        rotating = (self.align_yaw_to_tag and self._yaw_locked 
                    and yaw_err > math.radians(3.0))

        if in_7m_hover:
            self._descent_z_sp = self.YAW_LOCK_ALT
            self._descent_drift_count = 0  # Bỏ qua drift check khi đang chủ động hover ở 7m

            if not self._yaw_locked:
                if self._target_counter % 15 == 0:
                    self.get_logger().info(
                        f"YAW-SAMPLING tại {self.pos_enu[2]:.1f}m: "
                        f"{len(self._yaw_lock_buf)}/{self.YAW_LOCK_SAMPLES} mẫu"
                    )
            elif rotating:
                if self._target_counter % 15 == 0:
                    self.get_logger().info(
                        f"YAW-ALIGN (ROTATING): err={math.degrees(yaw_err):.1f}° — giữ nguyên XY"
                    )
            else:
                # Đã xoay xong yaw, bắt đầu căn chỉnh lại XY trước khi hạ tiếp
                xy_centered = self.target_rel_norm <= dr
                if xy_centered:
                    self._realign_cnt += 1
                    if self._realign_cnt >= 15:  # ổn định trong 15 mẫu (~0.5s)
                        self._yaw_realign_complete = True
                        self.get_logger().info("YAW-ALIGN & RE-CENTERING COMPLETE — bắt đầu hạ tiếp")
                else:
                    self._realign_cnt = 0

                if self._target_counter % 15 == 0:
                    self.get_logger().info(
                        f"YAW-ALIGN (RE-CENTERING): err={self.target_rel_norm:.2f}m (gate={dr:.2f}m), "
                        f"stable_cnt={self._realign_cnt}/15"
                    )
        else:
            if descent_ok:
                self._descent_drift_count = 0
                self._descent_z_sp = max(
                    self.FINAL_ALT,
                    self._descent_z_sp - self.DESCENT_RATE / self.CTRL_HZ,
                )
            else:
                # Z-LOCK: freeze altitude when drifting
                self._descent_drift_count += 1
                self._descent_z_sp = float(self.pos_enu[2])
                if self._target_counter % self.CTRL_HZ == 0:
                    self.get_logger().warn(
                        f"DESCENT Z-LOCK: err={self.target_rel_norm:.2f} > gate={dr:.2f}"
                    )
                if self._descent_drift_count >= self.ALIGN_CONFIRM:
                    if self.pos_enu[2] > self.LOW_ALT_COMMIT:
                        self._search_start = self._now()
                        self._transition("SEARCH")
                    else:
                        self._align_start = self._now()
                        self._centered_count = 0
                        self._transition("HORIZONTAL_APPROACH")
                    return

        # Thiết lập setpoint
        if rotating:
            # Giữ nguyên setpoint XY cũ, chỉ cập nhật độ cao mục tiêu Z
            self.sp_enu[2] = self._descent_z_sp
        else:
            self.sp_enu = self._visual_sp(self._descent_z_sp, self.MAX_DESCENT_STEP)

        if self._target_counter % self.CTRL_HZ == 0:
            phase = "descending" if descent_ok else "z-locked"
            self.get_logger().info(
                f"DESCEND ({phase}): alt={self._alt():.2f} z_sp={self._descent_z_sp:.2f} "
                f"err={self.target_rel_norm:.2f} gate={dr:.2f}"
            )
        self._target_counter += 1

        if self.pos_enu[2] <= self.FINAL_ALT + 0.05:
            self.get_logger().info(f"Final altitude reached ({self.FINAL_ALT:.2f}m)")
            self._transition("FINAL_APPROACH")

    def _st_final_approach(self):
        """Hand control back to PX4 AUTO.LAND for the last few cm."""
        self.get_logger().info("Switching to AUTO.LAND for final touchdown")
        self._set_mode("AUTO.LAND")
        self._transition("DONE")

    def _st_search(self):
        """Về đúng độ cao detect và tìm tag (KHÔNG leo cao khỏi tầm nhìn)."""
        search_alt = min(self.SEARCH_ALT, self.SEARCH_ALT_MAX)
        # Ưu tiên quay lại XY tag lần cuối; nếu chưa có thì dùng điểm takeover.
        if self.target_enu is not None:
            anchor = self.target_enu[:2]
        elif self._land_hold_pos is not None:
            anchor = self._land_hold_pos[:2]
        else:
            anchor = self.pos_enu[:2]
        self.sp_enu = np.array([float(anchor[0]), float(anchor[1]), search_alt])
        self._search_cnt += 1

        if self._target_fresh() and self.tracking_count >= self.TRACKING_CONFIRM:
            self._approach_alt = float(self.pos_enu[2])
            self._align_start = self._now()
            self._target_counter = 0
            self._centered_count = 0
            self._transition("HORIZONTAL_APPROACH")
            return

        if self._search_start and (self._now() - self._search_start) > self.SEARCH_TIMEOUT:
            self.get_logger().warn("Search timeout")
            if self._search_cnt >= self.MAX_SEARCH:
                self._transition("FALLBACK")
            else:
                self._search_start = self._now()

    def _st_target_lost(self):
        """Hold position briefly, try to reacquire target."""
        if self._target_lost_start is None:
            self._target_lost_start = self._now()

        if self._target_fresh() and self.tracking_count >= self.TRACKING_CONFIRM:
            resume = self._target_lost_from or "HORIZONTAL_APPROACH"
            self.get_logger().info(f"Target reacquired → {resume}")
            self._target_lost_start = None
            self._target_counter = 0
            self._centered_count = 0
            if resume == "HORIZONTAL_APPROACH":
                self._align_start = self._now()
            self._transition(resume)
            return

        elapsed = self._now() - self._target_lost_start
        # Hold current position
        self.sp_enu = self.pos_enu.copy()

        if elapsed > self.TARGET_LOSS_GRACE:
            if self.pos_enu[2] > self.LOW_ALT_COMMIT:
                self._search_start = self._now()
                self._transition("SEARCH")
            else:
                self.get_logger().warn("Target lost near ground → FINAL_APPROACH")
                self._transition("FINAL_APPROACH")

    def _st_fallback(self):
        """Give up precision landing, revert to AUTO.LAND."""
        self.get_logger().warn("Fallback → reverting to AUTO.LAND (GPS landing)")
        self._set_mode("AUTO.LAND")
        self._transition("DONE")

    def _st_done(self):
        """Landing complete or handed back to PX4."""
        if not self.armed:
            self.get_logger().info("LANDING COMPLETE — disarmed")
            self._transition("IDLE")


def main(args=None):
    rclpy.init(args=args)
    node = OffboardPreclandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
