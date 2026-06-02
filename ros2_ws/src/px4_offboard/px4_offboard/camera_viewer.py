#!/usr/bin/env python3
"""
camera_viewer.py — Xem camera feed từ x500_depth
px4_offboard package | ros2 run px4_offboard camera_viewer
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.frame_rgb   = None
        self.frame_depth = None

        self.create_subscription(Image,      '/camera',       self._rgb_cb,   10)
        self.create_subscription(Image,      '/depth_camera', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera_info',  self._info_cb,  10)
        self.create_timer(0.033, self._display)

        cv2.namedWindow('RGB  (IMX214 1920×1080)',   cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth (StereoOV7251 640×480)', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RGB  (IMX214 1920×1080)',    960, 540)
        cv2.resizeWindow('Depth (StereoOV7251 640×480)', 640, 480)
        self.get_logger().info('📷 Camera viewer ready — waiting for frames...')

    def _rgb_cb(self, msg):
        try:
            self.frame_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
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

    def _display(self):
        if self.frame_rgb is not None:
            small = cv2.resize(self.frame_rgb, (960, 540))
            cv2.imshow('RGB  (IMX214 1920×1080)', small)
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
