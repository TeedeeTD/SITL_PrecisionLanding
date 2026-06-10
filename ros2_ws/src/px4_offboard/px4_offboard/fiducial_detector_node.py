#!/usr/bin/env python3
"""Detect ArUco fiducials and publish camera-frame 6D landing targets."""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from dib_msgs.msg import LandingTarget6D, LandingTarget6DArray
from rclpy.node import Node
from sensor_msgs.msg import Image


def dictionary_constant(name: str) -> int:
    if hasattr(aruco, name):
        return int(getattr(aruco, name))

    matches = [candidate for candidate in dir(aruco) if candidate.lower() == name.lower()]
    if matches:
        return int(getattr(aruco, matches[0]))

    raise ValueError(f"OpenCV aruco dictionary not found: {name}")


class ArucoDetectorCompat:
    def __init__(self, dictionary, params):
        self.dictionary = dictionary
        self.params = params
        self.detector = aruco.ArucoDetector(dictionary, params) if hasattr(aruco, "ArucoDetector") else None

    def detectMarkers(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return aruco.detectMarkers(gray, self.dictionary, parameters=self.params)


def aruco_dictionary(dictionary_name: str):
    dictionary_id = dictionary_constant(dictionary_name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


def detector_parameters():
    if hasattr(aruco, "DetectorParameters"):
        return aruco.DetectorParameters()
    return aruco.DetectorParameters_create()


def make_detector(dictionary_name: str) -> ArucoDetectorCompat:
    params = detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.minMarkerPerimeterRate = 0.001
    params.maxMarkerPerimeterRate = 4.0
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 153
    params.adaptiveThreshWinSizeStep = 10
    return ArucoDetectorCompat(aruco_dictionary(dictionary_name), params)


def camera_matrix(image_width: int, image_height: int, horizontal_fov_rad: float) -> np.ndarray:
    fx = (image_width / 2.0) / math.tan(horizontal_fov_rad / 2.0)
    fy = fx
    cx = image_width / 2.0
    cy = image_height / 2.0
    return np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )


def parse_marker_sizes(
    marker_ids: Iterable[int],
    marker_sizes_m: Iterable[float],
    fallback_size_m: float,
) -> Dict[int, float]:
    ids = [int(marker_id) for marker_id in marker_ids]
    sizes = [float(marker_size) for marker_size in marker_sizes_m]
    if not ids and not sizes:
        return {}
    if len(ids) != len(sizes):
        raise ValueError("marker_ids and marker_sizes_m must have the same length")
    if any(size <= 0.0 for size in sizes) or fallback_size_m <= 0.0:
        raise ValueError("marker sizes must be positive")
    return dict(zip(ids, sizes))


def rotation_matrix_to_rpy(rotation: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


def estimate_marker_pose(
    corners: np.ndarray,
    marker_size_m: float,
    cam_mtx: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[np.ndarray, Tuple[float, float, float]]:
    half = marker_size_m / 2.0
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    image_points = corners.reshape((4, 2)).astype(np.float32)
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        cam_mtx,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        raise RuntimeError("solvePnP failed for detected ArUco marker")

    rotation, _ = cv2.Rodrigues(rvec)
    return tvec.reshape(3), rotation_matrix_to_rpy(rotation)


def detect_fiducials(
    gray: np.ndarray,
    detector: ArucoDetectorCompat,
    cam_mtx: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_by_id: Dict[int, float],
    fallback_marker_size_m: float,
) -> List[dict]:
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []

    targets = []
    for marker_corners, marker_id_raw in zip(corners, ids.flatten()):
        marker_id = int(marker_id_raw)
        marker_size_m = marker_size_by_id.get(marker_id, fallback_marker_size_m)
        tvec, rpy = estimate_marker_pose(marker_corners, marker_size_m, cam_mtx, dist_coeffs)
        targets.append(
            {
                "tag_id": marker_id,
                "size_m": marker_size_m,
                "x": float(tvec[0]),
                "y": float(tvec[1]),
                "z": float(tvec[2]),
                "roll": float(rpy[0]),
                "pitch": float(rpy[1]),
                "yaw": float(rpy[2]),
            }
        )

    targets.sort(key=lambda target: target["tag_id"])
    return targets


class FiducialDetectorNode(Node):
    def __init__(self):
        super().__init__("fiducial_detector_node")

        self.declare_parameter("image_topic", "/gimbal_camera")
        self.declare_parameter("output_topic", "/landing/targets_camera")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_size_m", 1.0)
        self.declare_parameter("marker_ids", [10, 11, 12])
        self.declare_parameter("marker_sizes_m", [1.0, 0.22, 0.08])
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("horizontal_fov_rad", 2.0)

        image_topic = str(self.get_parameter("image_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        dictionary_name = str(self.get_parameter("dictionary").value)
        self.fallback_marker_size_m = float(self.get_parameter("marker_size_m").value)
        marker_ids = self.get_parameter("marker_ids").value
        marker_sizes_m = self.get_parameter("marker_sizes_m").value
        image_width = int(self.get_parameter("image_width").value)
        image_height = int(self.get_parameter("image_height").value)
        horizontal_fov_rad = float(self.get_parameter("horizontal_fov_rad").value)

        self.detector = make_detector(dictionary_name)
        self.marker_size_by_id = parse_marker_sizes(
            marker_ids,
            marker_sizes_m,
            self.fallback_marker_size_m,
        )
        self.cam_mtx = camera_matrix(image_width, image_height, horizontal_fov_rad)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(LandingTarget6DArray, output_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f"Fiducial detector ready: image={image_topic}, output={output_topic}, "
            f"dictionary={dictionary_name}, marker_sizes={self.marker_size_by_id}"
        )

    def _on_image(self, msg: Image):
        output = LandingTarget6DArray()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.camera_frame

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detect_fiducials(
                gray,
                self.detector,
                self.cam_mtx,
                self.dist_coeffs,
                self.marker_size_by_id,
                self.fallback_marker_size_m,
            )
        except Exception as exc:
            self.get_logger().warn(f"fiducial detection failed: {exc}", throttle_duration_sec=2.0)
            self.publisher.publish(output)
            return

        for detection in detections:
            target = LandingTarget6D()
            target.header = output.header
            target.x = detection["x"]
            target.y = detection["y"]
            target.z = detection["z"]
            target.roll = detection["roll"]
            target.pitch = detection["pitch"]
            target.yaw = detection["yaw"]
            target.state = LandingTarget6D.TRACKING
            target.tag_id = detection["tag_id"]
            output.targets.append(target)

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FiducialDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
