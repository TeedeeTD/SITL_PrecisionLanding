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
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamGet


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
    YAW_LOCK_SAMPLES = 8       # số mẫu gộp trước khi CHỐT heading đích rồi đóng băng

    def __init__(self) -> None:
        super().__init__("offboard_precland_controller")

        self.declare_parameter("camera_x_to_body_east_sign", 1.0)
        self.declare_parameter("camera_y_to_body_north_sign", -1.0)
        self.declare_parameter("camera_yaw_frame", "body")
        self.declare_parameter("camera_offset_x", 0.1517)
        self.declare_parameter("camera_offset_y", 0.0)
        self.declare_parameter("marker_size", 0.50)
        self.declare_parameter("target_topic", "/landing/target_camera")
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
        self.create_subscription(LandingTarget6D, target_topic, self._on_target, 10)
        self.create_subscription(State, "/mavros/state", self._on_state, state_qos)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_ext_state, state_qos)

        # Services
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cmd_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.param_get_client = self.create_client(ParamGet, "/mavros/param/get")

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
        self.mavros_connected = msg.connected
        was = self.is_landing
        self.is_landing = (msg.mode == "AUTO.LAND")
        if self.is_landing != was:
            self.get_logger().info(f"Landing flag: {self.is_landing} (mode={msg.mode})")

    def _on_ext_state(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state
        if not self.is_landing and msg.landed_state == ExtendedState.LANDED_STATE_LANDING:
            self.is_landing = True

    def _on_target(self, msg: LandingTarget6D) -> None:
        if msg.tag_id < 0 or msg.state == LandingTarget6D.LOST:
            self.target_samples.clear()
            self.filtered_rel_enu = None
            return

        self.tracking_count += 1
        tvec = np.array([msg.x, msg.y, msg.z])
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
        self.target_samples.append(abs_xy.copy())
        stacked = np.stack(tuple(self.target_samples), axis=0)
        med = np.median(stacked, axis=0)
        a = self._alpha()
        if self.filtered_rel_enu is None:
            self.filtered_rel_enu = med.copy()
        else:
            self.filtered_rel_enu = (1.0 - a) * self.filtered_rel_enu + a * med

        self.target_enu = self.filtered_rel_enu.copy()
        self.target_rel_norm = float(np.linalg.norm(self.target_enu - self.pos_enu[:2]))
        self.last_pose_time = self._now()

        # Heading ĐÍCH tuyệt đối (ENU) = body_yaw(h_q, đồng bộ trễ) + yaw tương đối tag.
        # CHỐT MỘT LẦN rồi ĐÓNG BĂNG: lúc chốt drone chưa xoay nên phép đo còn hợp lệ;
        # đóng băng biến mục tiêu thành hằng số → không có vòng hồi tiếp để spin, kể cả
        # khi gimbal khóa yaw theo world.
        ty = getattr(msg, "yaw", None)
        if (
            ty is not None
            and self.align_yaw_to_tag
            and not self._yaw_locked
            and self.tracking_count >= self.TRACKING_CONFIRM
        ):
            body_yaw = self._yaw(h_q)
            world_yaw = self._wrap(
                body_yaw + self.tag_yaw_sign * float(ty) + self.tag_yaw_offset
            )
            self._yaw_lock_buf.append(world_yaw)
            if len(self._yaw_lock_buf) >= self.YAW_LOCK_SAMPLES:
                self._tag_yaw_abs = self._circmean(self._yaw_lock_buf)
                self._yaw_locked = True
                self.get_logger().info(
                    f"[YAW-LOCK] chốt target={math.degrees(self._tag_yaw_abs):+.1f}° "
                    f"(mẫu cuối tag_yaw={math.degrees(float(ty)):+.1f}° body={math.degrees(body_yaw):+.1f}° "
                    f"sign={self.tag_yaw_sign:+.0f} off={math.degrees(self.tag_yaw_offset):+.1f}°) — đóng băng"
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
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _circmean(angles):
        s = sum(math.sin(a) for a in angles)
        c = sum(math.cos(a) for a in angles)
        return math.atan2(s, c)

    def _desired_yaw(self):
        """Giữ nguyên heading lúc takeover cho tới khi tag được khóa chắc chắn.
        Khi đó slew về heading ĐÍCH TUYỆT ĐỐI (hằng số) — KHÔNG cộng heading sống,
        nếu không latency vision sẽ làm mục tiêu trôi và drone xoay mòng mòng."""
        aligning = (
            self.align_yaw_to_tag
            and self._yaw_locked
            and self._tag_yaw_abs is not None
            and self.state in ("HORIZONTAL_APPROACH", "DESCEND_ABOVE_TARGET")
        )
        if not aligning:
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

    # ── Mode switching ────────────────────────────────────────

    def _set_mode(self, mode: str):
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def _query_px4_params(self):
        """Query PX4 parameter RTL_PLD_MD to automatically sync the precision landing mode."""
        if not self.param_get_client.service_is_ready():
            return
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
        if self.precland_mode == 0:
            return  # Precision landing disabled — không can thiệp

        if not (self.is_landing and self.armed):
            return

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
        """Move horizontally over target, hold altitude."""
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
        """Descend with Z-lock when drifting off center."""
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
