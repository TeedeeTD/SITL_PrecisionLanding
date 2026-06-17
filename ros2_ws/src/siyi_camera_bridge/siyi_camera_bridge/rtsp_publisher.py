#!/usr/bin/env python3
"""
SIYI A8 Mini RTSP → ROS2 Image bridge node.

Reads RTSP stream from SIYI A8 Mini camera via OpenCV,
optionally flips the image 180° (camera mounted upside-down),
and publishes sensor_msgs/Image + sensor_msgs/CameraInfo.
"""

import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Set RTSP TCP transport BEFORE importing cv2 / opening capture
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
import cv2  # noqa: E402


class RtspPublisher(Node):
    """ROS2 node that bridges an RTSP camera stream to Image + CameraInfo topics."""

    def __init__(self):
        super().__init__('siyi_rtsp_publisher')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('rtsp_url',
                               'rtsp://192.168.168.14:8554/main.264')
        self.declare_parameter('frame_id', 'siyi_camera_optical_frame')
        self.declare_parameter('flip_180', True)
        self.declare_parameter('target_fps', 30.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        # Fallback intrinsics matching SIYI A8 Mini HFOV ≈ 81°
        # fx = fy = (1280 / 2) / tan(81° / 2) = 640 / tan(40.5°) ≈ 749.338
        self.declare_parameter('camera_fx', 749.338)
        self.declare_parameter('camera_fy', 749.338)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 360.0)

        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.frame_id = self.get_parameter('frame_id').value
        self.flip_180 = self.get_parameter('flip_180').value
        self.target_fps = self.get_parameter('target_fps').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.camera_fx = self.get_parameter('camera_fx').value
        self.camera_fy = self.get_parameter('camera_fy').value
        self.camera_cx = self.get_parameter('camera_cx').value
        self.camera_cy = self.get_parameter('camera_cy').value

        # ── Publishers ──────────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, '/siyi/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/siyi/camera_info', 10)

        # ── Bridge ──────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Build static CameraInfo message ─────────────────────────
        self.camera_info_msg = self._build_camera_info()

        # ── Open RTSP capture ───────────────────────────────────────
        self.cap = None
        self._open_capture()

        # ── Timer for frame grabbing ────────────────────────────────
        timer_period = 1.0 / self.target_fps
        self.timer = self.create_timer(timer_period, self._timer_callback)

        # ── Stats ───────────────────────────────────────────────────
        self.frame_count = 0
        self.fail_count = 0
        self.max_consecutive_fails = 30  # Reconnect after this many failures

        self.get_logger().info(
            f'SIYI RTSP Publisher started: url={self.rtsp_url} '
            f'flip_180={self.flip_180} target_fps={self.target_fps} '
            f'resolution={self.image_width}x{self.image_height}'
        )

    def _open_capture(self):
        """Open or reopen the RTSP VideoCapture."""
        if self.cap is not None:
            self.cap.release()

        self.get_logger().info(f'Opening RTSP stream: {self.rtsp_url}')
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open RTSP stream: {self.rtsp_url}. '
                'Will retry on next timer tick.'
            )
            return False

        # Try to set resolution (RTSP source may ignore this)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        # Minimize buffer to reduce latency
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.fail_count = 0
        self.get_logger().info('RTSP stream opened successfully')
        return True

    def _build_camera_info(self) -> CameraInfo:
        """Build a CameraInfo message with fallback intrinsics."""
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Intrinsic camera matrix (K)
        fx = self.camera_fx
        fy = self.camera_fy
        cx = self.camera_cx
        cy = self.camera_cy
        msg.k = [
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0,
        ]

        # Rectification matrix (identity for monocular)
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # Projection matrix (P)
        msg.p = [
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        return msg

    def _timer_callback(self):
        """Grab a frame from RTSP and publish."""
        if self.cap is None or not self.cap.isOpened():
            self.fail_count += 1
            if self.fail_count % 30 == 1:
                self.get_logger().warn(
                    f'RTSP not open, attempting reconnect... '
                    f'(fail_count={self.fail_count})'
                )
            self._open_capture()
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.fail_count += 1
            if self.fail_count >= self.max_consecutive_fails:
                self.get_logger().warn(
                    f'Lost RTSP stream after {self.fail_count} failures, reconnecting...'
                )
                self._open_capture()
            return

        self.fail_count = 0
        self.frame_count += 1

        # Flip 180° if camera is mounted upside-down
        if self.flip_180:
            frame = cv2.flip(frame, -1)

        # Timestamp
        stamp = self.get_clock().now().to_msg()

        # Publish Image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Publish CameraInfo (same timestamp)
        self.camera_info_msg.header.stamp = stamp
        self.info_pub.publish(self.camera_info_msg)

        # Log stats periodically
        if self.frame_count % (int(self.target_fps) * 5) == 0:
            self.get_logger().info(
                f'Published {self.frame_count} frames '
                f'({frame.shape[1]}x{frame.shape[0]}) '
                f'flip_180={self.flip_180}'
            )

    def destroy_node(self):
        """Clean up on shutdown."""
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info('RTSP capture released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtspPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
