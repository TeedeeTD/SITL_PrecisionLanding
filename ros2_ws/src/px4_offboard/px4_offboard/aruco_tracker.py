#!/usr/bin/env python3
"""ArUco tracker node.

Subscribes to camera image/info and drone pose/state,
detects the target ArUco marker using OpenCV, estimates its 3D pose,
performs body-to-ENU coordinate transformations, and publishes:
1. /aruco_tracker/pose (geometry_msgs/msg/PoseStamped)
2. /landing/annotated_image (sensor_msgs/msg/Image)
"""

import math
import time
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


class ArucoDetectorCompat:
    def __init__(self, dictionary, params):
        self.dictionary = dictionary
        self.params = params
        self.detector = aruco.ArucoDetector(dictionary, params) if hasattr(aruco, "ArucoDetector") else None

    def detectMarkers(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return aruco.detectMarkers(gray, self.dictionary, parameters=self.params)


def dictionary_constant(name: str) -> int:
    if hasattr(aruco, name):
        return int(getattr(aruco, name))
    matches = [candidate for candidate in dir(aruco) if candidate.lower() == name.lower()]
    if matches:
        return int(getattr(aruco, matches[0]))
    raise ValueError(f"OpenCV aruco dictionary not found: {name}")


def aruco_dictionary(dictionary_name: str):
    dictionary_id = dictionary_constant(dictionary_name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__("aruco_tracker_node")

        self.declare_parameter("image_topic", "/gimbal_camera")
        self.declare_parameter("camera_info_topic", "/gimbal_camera/camera_info")
        self.declare_parameter("pose_output_topic", "/aruco_tracker/pose")
        self.declare_parameter("annotated_image_topic", "/landing/annotated_image")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("marker_size", 0.50)
        self.declare_parameter("camera_frame", "camera_link")

        # Coordinate transformation parameters (to match C++ fractal tracker)
        self.declare_parameter("camera_x_to_body_east_sign", 1.0)
        self.declare_parameter("camera_y_to_body_north_sign", -1.0)
        self.declare_parameter("camera_offset_x", 0.1517)
        self.declare_parameter("camera_offset_y", 0.0)

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        pose_topic = self.get_parameter("pose_output_topic").value
        annotated_topic = self.get_parameter("annotated_image_topic").value
        dict_name = self.get_parameter("dictionary").value
        self.target_tag_id = int(self.get_parameter("target_tag_id").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.camera_frame = self.get_parameter("camera_frame").value

        self.camera_x_to_east_sign = float(self.get_parameter("camera_x_to_body_east_sign").value)
        self.camera_y_to_north_sign = float(self.get_parameter("camera_y_to_body_north_sign").value)
        self.camera_offset_x = float(self.get_parameter("camera_offset_x").value)
        self.camera_offset_y = float(self.get_parameter("camera_offset_y").value)

        # Initialize detector
        aruco_dict = aruco_dictionary(dict_name)
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = 0.01
        self.detector = ArucoDetectorCompat(aruco_dict, params)

        self.bridge = CvBridge()
        self.cam_mtx = None
        self.dist_coeffs = None

        # State tracking for HUD (matching C++ tracker)
        self.last_uav_pose = None
        self.last_lander_state = "UNKNOWN"
        self.last_proc_time = 0.0
        self.min_interval = 0.05  # Throttle to max 20Hz

        # Latency & FPS profiling variables
        self.last_processing_latency_ms = 0.0
        self.last_source_latency_ms = 0.0
        self.source_latency_valid = False
        self.current_fps = 0.0
        self.fps_frame_count = 0
        self.last_fps_time = self.get_clock().now()
        self.last_detected_ids_str = "None"

        # Subscriptions
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

        # Subscriber for local position (QoS BestEffort, volatile durability, depth 1)
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_drone_pose, pose_qos)
        self.create_subscription(String, "/lander/state", self._on_lander_state, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.image_pub = self.create_publisher(Image, annotated_topic, 10)

        self.get_logger().info(
            f"ArUco Tracker Node ready: dict={dict_name}, tag_id={self.target_tag_id}, size={self.marker_size}m"
        )

    def _on_camera_info(self, msg: CameraInfo):
        if self.cam_mtx is not None:
            return
        self.cam_mtx = np.array(msg.k, dtype=np.float32).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)
        self.get_logger().info(f"Received camera info calibration: fx={self.cam_mtx[0,0]:.2f}")

    def _on_drone_pose(self, msg: PoseStamped):
        self.last_uav_pose = msg

    def _on_lander_state(self, msg: String):
        self.last_lander_state = msg.data

    def _on_image(self, msg: Image):
        now_sec = time.time()
        if now_sec - self.last_proc_time < self.min_interval:
            return
        self.last_proc_time = now_sec

        # Fallback nominal camera matrix if info topic not yet received
        if self.cam_mtx is None:
            w, h = msg.width, msg.height
            # Nominal HFOV = 1.2 rad
            fx = (w / 2.0) / math.tan(1.2 / 2.0)
            self.cam_mtx = np.array([
                [fx, 0.0, w / 2.0],
                [0.0, fx, h / 2.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)
            self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        start_time = time.time()

        # Update FPS
        now_clock = self.get_clock().now()
        self.fps_frame_count += 1
        elapsed = (now_clock - self.last_fps_time).nanoseconds / 1e9
        if elapsed >= 1.0:
            self.current_fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.last_fps_time = now_clock

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        # Double-pass fallback using Otsu thresholding if target tag not found
        target_found = False
        if ids is not None:
            ids_flat = ids.flatten().astype(int).tolist()
            if self.target_tag_id in ids_flat:
                target_found = True
                
        if not target_found:
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            corners_t, ids_t, rejected_t = self.detector.detectMarkers(thresh)
            if ids_t is not None:
                ids_flat_t = ids_t.flatten().astype(int).tolist()
                if self.target_tag_id in ids_flat_t:
                    corners = corners_t
                    ids = ids_t
                    rejected = rejected_t

        target_tvec = None
        target_rvec = None
        target_corners = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int).tolist()
            self.last_detected_ids_str = ",".join(map(str, ids_flat))
            for i, marker_id in enumerate(ids_flat):
                if marker_id == self.target_tag_id:
                    target_corners = corners[i]
                    success, rvec, tvec = self._estimate_marker_pose(target_corners)
                    if success:
                        target_tvec = tvec.flatten()
                        target_rvec = rvec.flatten()
                        
                        # Publish PoseStamped
                        pose_msg = PoseStamped()
                        pose_msg.header = msg.header
                        pose_msg.header.frame_id = self.camera_frame
                        pose_msg.pose.position.x = float(target_tvec[0])
                        pose_msg.pose.position.y = float(target_tvec[1])
                        pose_msg.pose.position.z = float(target_tvec[2])
                        
                        # Set orientation from rvec
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        qw, qx, qy, qz = self._rot_matrix_to_quaternion(rotation_matrix)
                        pose_msg.pose.orientation.w = qw
                        pose_msg.pose.orientation.x = qx
                        pose_msg.pose.orientation.y = qy
                        pose_msg.pose.orientation.z = qz
                        
                        self.pose_pub.publish(pose_msg)
                    break
        else:
            self.last_detected_ids_str = "None"

        # Draw Annotations
        annotated = frame.copy()
        if target_corners is not None:
            aruco.drawDetectedMarkers(annotated, [target_corners], np.array([[self.target_tag_id]]))

        if target_tvec is not None and target_rvec is not None:
            axis_len = self.marker_size * 0.5
            cv2.drawFrameAxes(annotated, self.cam_mtx, self.dist_coeffs, target_rvec, target_tvec, axis_len)
            
            # Print info on marker
            cx = int(np.mean(target_corners[0][:, 0]))
            cy = int(np.mean(target_corners[0][:, 1]))
            cv2.putText(
                annotated,
                f"TGT ID:{self.target_tag_id} Z={target_tvec[2]:.2f}m",
                (cx + 10, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        else:
            # Draw "NOT FOUND" overlay
            cv2.putText(annotated, "NOT FOUND", (20, 30), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(annotated, "NOT FOUND", (20, 30), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 1, cv2.LINE_AA)

        # Calculate latency metrics
        self.last_processing_latency_ms = (time.time() - start_time) * 1000.0
        self.source_latency_valid = False
        
        # Calculate camera to tracker latency
        msg_time = Time.from_msg(msg.header.stamp)
        if msg_time.nanoseconds > 0:
            source_latency = (self.get_clock().now() - msg_time).nanoseconds / 1e6
            if -1.0 <= source_latency < 60000.0:
                self.last_source_latency_ms = source_latency
                self.source_latency_valid = True

        # Draw overlays exactly like C++ tracker
        self._draw_latency_overlay(annotated)
        self._draw_flight_state_overlay(annotated, target_tvec)

        # Publish image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            img_msg.header = msg.header
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    def _estimate_marker_pose(self, corners: np.ndarray):
        half = self.marker_size / 2.0
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
            self.cam_mtx,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        return success, rvec, tvec

    def _rot_matrix_to_quaternion(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return qw, qx, qy, qz

    def _draw_latency_overlay(self, image: np.ndarray):
        font_face = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 1
        margin = 10
        line_height = 22
        panel_height = 3 * line_height + 12
        panel_top = max(0, image.shape[0] - panel_height - margin)
        panel_right = min(image.shape[1] - margin, 510)

        # Draw black solid panel
        cv2.rectangle(
            image, (margin, panel_top), (panel_right, image.shape[0] - margin),
            (0, 0, 0), -1
        )

        processing_text = f"Detector processing: {self.last_processing_latency_ms:.1f} ms"
        if self.last_processing_latency_ms > 100.0:
            processing_text += "  [WARN]"
        processing_color = (80, 220, 80) if self.last_processing_latency_ms <= 100.0 else (0, 80, 255)

        cv2.putText(
            image, processing_text, (margin + 8, panel_top + line_height),
            font_face, font_scale, processing_color, thickness, cv2.LINE_AA
        )

        source_text = "Camera -> tracker: N/A (clock mismatch)"
        if self.source_latency_valid:
            source_text = f"Camera -> tracker: {self.last_source_latency_ms:.1f} ms"
            if self.last_source_latency_ms > 100.0:
                source_text += "  [WARN]"
        source_color = (0, 200, 255) if not self.source_latency_valid else (
            (80, 220, 80) if self.last_source_latency_ms <= 100.0 else (0, 80, 255)
        )

        cv2.putText(
            image, source_text, (margin + 8, panel_top + 2 * line_height),
            font_face, font_scale, source_color, thickness, cv2.LINE_AA
        )

        info_text = f"Tracker FPS: {self.current_fps:.1f} Hz | Detected IDs: {self.last_detected_ids_str}"
        cv2.putText(
            image, info_text, (margin + 8, panel_top + 3 * line_height),
            font_face, font_scale, (255, 255, 255), thickness, cv2.LINE_AA
        )

    def _draw_flight_state_overlay(self, image: np.ndarray, target_tvec):
        font_face = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 1
        line_h = 22
        margin = 10
        panel_w = 400
        panel_h = 6 * line_h + 12
        panel_top = max(0, image.shape[0] - panel_h - margin)
        panel_left = max(0, image.shape[1] - panel_w - margin)

        # Draw solid black background panel
        cv2.rectangle(
            image, (panel_left, panel_top), (image.shape[1] - margin, image.shape[0] - margin),
            (0, 0, 0), -1
        )
        # Draw white border around the panel
        cv2.rectangle(
            image, (panel_left, panel_top), (image.shape[1] - margin, image.shape[0] - margin),
            (150, 150, 150), 1
        )

        pos = [panel_left + 10, panel_top + line_h]

        def draw_text(text: str, color=(255, 255, 255)):
            cv2.putText(image, text, (pos[0], pos[1]), font_face, font_scale, color, thickness, cv2.LINE_AA)
            pos[1] += line_h

        draw_text("FLIGHT STATE: " + self.last_lander_state, (80, 220, 240))

        if self.last_uav_pose is not None:
            uav_x = self.last_uav_pose.pose.position.x
            uav_y = self.last_uav_pose.pose.position.y
            uav_z = self.last_uav_pose.pose.position.z

            # Compute yaw from quaternion
            qx = self.last_uav_pose.pose.orientation.x
            qy = self.last_uav_pose.pose.orientation.y
            qz = self.last_uav_pose.pose.orientation.z
            qw = self.last_uav_pose.pose.orientation.w
            yaw = quaternion_to_yaw(qx, qy, qz, qw)

            draw_text(f"UAV ENU: E={uav_x:.2f}, N={uav_y:.2f}, U={uav_z:.2f}")
            draw_text(f"UAV YAW: {math.degrees(yaw):.1f} deg")

            if target_tvec is not None:
                tx, ty, tz = target_tvec[0], target_tvec[1], target_tvec[2]
                east_body = self.camera_x_to_east_sign * tx
                north_body = self.camera_y_to_north_sign * ty

                x_body = north_body + self.camera_offset_x
                y_body = -east_body + self.camera_offset_y

                c = math.cos(yaw)
                s = math.sin(yaw)

                rel_east = x_body * c - y_body * s
                rel_north = x_body * s + y_body * c

                abs_east = uav_x + rel_east
                abs_north = uav_y + rel_north

                draw_text(f"REL ENU: E={rel_east:.2f}, N={rel_north:.2f}", (100, 255, 100))
                draw_text(f"TGT ENU: E={abs_east:.2f}, N={abs_north:.2f}", (100, 100, 255))
                draw_text(f"CAM TVEC: [{tx:.2f}, {ty:.2f}, {tz:.2f}]")
            else:
                draw_text("REL ENU: NO MARKER DETECTED", (0, 0, 255))
                draw_text("TGT ENU: NO MARKER DETECTED", (0, 0, 255))
                draw_text("CAM TVEC: N/A")
        else:
            draw_text("UAV ENU: WAITING FOR MAVROS...", (0, 150, 255))
            draw_text("UAV YAW: WAITING FOR MAVROS...", (0, 150, 255))
            draw_text("REL ENU: WAITING FOR MAVROS...")
            draw_text("TGT ENU: WAITING FOR MAVROS...")
            draw_text("CAM TVEC: N/A")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down tracker...")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
