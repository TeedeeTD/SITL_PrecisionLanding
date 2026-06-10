#!/usr/bin/env python3
"""
camera_viewer.py — View camera feed and visualize detected ArUco tags.
px4_offboard package | ros2 run px4_offboard camera_viewer
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np


NESTED_TAGS = {
    10: ('outer', 1.00),
    11: ('middle', 0.22),
    12: ('inner', 0.08),
}


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.frame_rgb = None
        self.frame_depth = None
        self.last_ids = []
        self.last_selected_id = None

        self.declare_parameter('image_topic', '/gimbal_camera')
        self.declare_parameter('depth_topic', '/depth_camera')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco_overlay', True)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.aruco_overlay = bool(self.get_parameter('aruco_overlay').value)
        dictionary_name = self.get_parameter('aruco_dictionary').value
        self.aruco_detector = self._create_aruco_detector(dictionary_name)

        self.create_subscription(Image, image_topic, self._rgb_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        self.create_timer(0.033, self._display)

        cv2.namedWindow('RGB  (ArUco overlay)', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth (StereoOV7251 640×480)', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RGB  (ArUco overlay)', 960, 540)
        cv2.resizeWindow('Depth (StereoOV7251 640×480)', 640, 480)
        self.get_logger().info(
            f'📷 Camera viewer ready — image={image_topic}, aruco={dictionary_name}')

    def _create_aruco_detector(self, dictionary_name):
        if not hasattr(aruco, dictionary_name):
            self.get_logger().warn(
                f'Unknown ArUco dictionary {dictionary_name}; overlay disabled')
            self.aruco_overlay = False
            return None

        dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = 0.001
        params.maxMarkerPerimeterRate = 4.0

        if hasattr(aruco, 'ArucoDetector'):
            return aruco.ArucoDetector(dictionary, params)

        return (dictionary, params)

    def _rgb_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.aruco_overlay and self.aruco_detector is not None:
                frame = self._annotate_aruco(frame)
            self.frame_rgb = frame
        except Exception as e:
            self.get_logger().warn(f'RGB: {e}')

    def _depth_cb(self, msg):
        try:
            d = self.bridge.imgmsg_to_cv2(msg, passthrough=True).astype(np.float32)
            norm = (np.clip(d / 10.0, 0, 1) * 255).astype(np.uint8)
            colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            h, w = d.shape
            cx, cy = w // 2, h // 2
            center_d = d[cy, cx]
            cv2.drawMarker(colored, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            label = f"Center: {center_d:.2f}m" if np.isfinite(center_d) else "Center: inf"
            cv2.putText(colored, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            self.frame_depth = colored
        except Exception as e:
            self.get_logger().warn(f'Depth: {e}')

    def _info_cb(self, msg):
        self.get_logger().info(
            f'[Info] {msg.width}×{msg.height} fx={msg.k[0]:.0f}', once=True)

    def _detect_aruco(self, gray):
        if hasattr(aruco, 'ArucoDetector'):
            return self.aruco_detector.detectMarkers(gray)

        dictionary, params = self.aruco_detector
        return aruco.detectMarkers(gray, dictionary, parameters=params)

    def _select_nested_target(self, ids):
        expected = [marker_id for marker_id in ids if marker_id in NESTED_TAGS]
        if not expected:
            return None

        return min(expected, key=lambda marker_id: NESTED_TAGS[marker_id][1])

    def _annotate_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self._detect_aruco(gray)
        id_list = [] if ids is None else ids.flatten().astype(int).tolist()
        selected_id = self._select_nested_target(id_list)
        self.last_ids = id_list
        self.last_selected_id = selected_id

        annotated = frame.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(annotated, corners, ids)

            for marker_corners, marker_id in zip(corners, id_list):
                pts = marker_corners.reshape(-1, 2).astype(int)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                name, size_m = NESTED_TAGS.get(marker_id, ('unknown', 0.0))
                label = f'ID {marker_id}'
                if marker_id in NESTED_TAGS:
                    label = f'ID {marker_id} {name} {size_m:.2f}m'
                color = (0, 255, 0) if marker_id == selected_id else (255, 200, 0)
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                cv2.putText(
                    annotated,
                    label,
                    (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )

        if selected_id is not None:
            name, size_m = NESTED_TAGS[selected_id]
            status = f'Detected: {id_list} | selected: {selected_id} {name} {size_m:.2f}m'
            color = (0, 255, 0)
        elif id_list:
            status = f'Detected: {id_list} | selected: none'
            color = (0, 220, 255)
        else:
            status = f'Detected: none | rejected: {len(rejected)}'
            color = (0, 220, 255)

        cv2.rectangle(annotated, (8, 8), (760, 44), (0, 0, 0), -1)
        cv2.putText(
            annotated,
            status,
            (18, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
        return annotated

    def _display(self):
        if self.frame_rgb is not None:
            small = cv2.resize(self.frame_rgb, (960, 540))
            cv2.imshow('RGB  (ArUco overlay)', small)
        if self.frame_depth is not None:
            cv2.imshow('Depth (StereoOV7251 640×480)', self.frame_depth)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == '__main__':
    main()
