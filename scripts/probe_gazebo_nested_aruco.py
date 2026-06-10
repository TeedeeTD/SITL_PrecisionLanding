#!/usr/bin/env python3
"""Probe nested ArUco detection from a Gazebo/ROS camera topic.

This is a Step 0 validation helper. It does not depend on px4_msgs and does not
command the vehicle. Use it after the simulator is running, the camera topic is
bridged, and the UAV/gimbal are positioned so the marker is visible.
"""

from __future__ import annotations

import argparse
import time
from collections import Counter
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def aruco_dictionary(dictionary_name: str):
    if not hasattr(aruco, dictionary_name):
        raise ValueError(f"OpenCV aruco dictionary not found: {dictionary_name}")
    dictionary_id = getattr(aruco, dictionary_name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


class ArucoDetectorCompat:
    def __init__(self, dictionary_name: str):
        params = aruco.DetectorParameters() if hasattr(aruco, "DetectorParameters") else aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = 0.001
        params.maxMarkerPerimeterRate = 4.0
        self.dictionary = aruco_dictionary(dictionary_name)
        self.detector = aruco.ArucoDetector(self.dictionary, params) if hasattr(aruco, "ArucoDetector") else None
        self.params = params

    def detect(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return aruco.detectMarkers(gray, self.dictionary, parameters=self.params)


def image_msg_to_bgr(msg: Image):
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    encoding = msg.encoding.lower()
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    row_width = msg.width * channels
    data = np.frombuffer(msg.data, dtype=np.uint8)
    rows = data.reshape((msg.height, msg.step))[:, :row_width]

    if channels == 1:
        return rows.reshape((msg.height, msg.width))

    image = rows.reshape((msg.height, msg.width, channels))
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


class NestedArucoProbe(Node):
    def __init__(self, args):
        super().__init__("nested_aruco_gazebo_probe")
        self.args = args
        self.expected_ids = set(args.expected_ids)
        self.detector = ArucoDetectorCompat(args.dictionary)
        self.id_counts: Counter[int] = Counter()
        self.frames = 0
        self.frames_with_expected = 0
        self.pass_frames = 0
        self.best_expected_count = 0
        self.best_ids: list[int] = []
        self.rejected_last = 0
        self.last_frame = None
        self.last_corners = None
        self.last_ids = None
        self.started = time.monotonic()
        self.last_log = self.started
        self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg: Image):
        self.frames += 1
        try:
            bgr = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detect(gray)
        self.last_frame = bgr.copy()
        self.last_corners = corners
        self.last_ids = ids
        self.rejected_last = len(rejected)
        id_list = [] if ids is None else ids.flatten().astype(int).tolist()

        if id_list:
            self.id_counts.update(id_list)

        expected_seen = sorted(self.expected_ids.intersection(id_list))
        if expected_seen:
            self.frames_with_expected += 1

        if len(expected_seen) >= self.args.pass_min_ids:
            self.pass_frames += 1

        if len(expected_seen) > self.best_expected_count:
            self.best_expected_count = len(expected_seen)
            self.best_ids = id_list
            self.save_best_frame(bgr, corners, ids)

        now = time.monotonic()
        if now - self.last_log >= self.args.log_interval:
            self.last_log = now
            print(
                "PROBE "
                f"t={now - self.started:.1f}s frames={self.frames} "
                f"ids={id_list if id_list else 'none'} rejected={self.rejected_last} "
                f"pass_frames={self.pass_frames}"
            )

    def save_best_frame(self, bgr, corners, ids):
        if not self.args.save_frame:
            return
        output = Path(self.args.save_frame)
        output.parent.mkdir(parents=True, exist_ok=True)
        annotated = bgr.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(annotated, corners, ids)
        cv2.imwrite(str(output), annotated)

    def save_last_frame(self):
        if not self.args.save_last_frame or self.last_frame is None:
            return
        output = Path(self.args.save_last_frame)
        output.parent.mkdir(parents=True, exist_ok=True)
        annotated = self.last_frame.copy()
        if self.last_ids is not None:
            aruco.drawDetectedMarkers(annotated, self.last_corners, self.last_ids)
        cv2.imwrite(str(output), annotated)

    def passed(self) -> bool:
        return self.pass_frames >= self.args.pass_min_frames

    def print_summary(self):
        self.save_last_frame()
        print()
        print(f"topic: {self.args.topic}")
        print(f"dictionary: {self.args.dictionary}")
        print(f"expected_ids: {sorted(self.expected_ids)}")
        print(f"frames: {self.frames}")
        print(f"id_counts: {dict(sorted(self.id_counts.items()))}")
        print(f"frames_with_expected: {self.frames_with_expected}")
        print(f"pass_frames: {self.pass_frames}")
        print(f"best_ids: {self.best_ids if self.best_ids else 'none'}")
        print(f"best_frame: {self.args.save_frame if self.args.save_frame else 'disabled'}")
        print(f"last_frame: {self.args.save_last_frame if self.args.save_last_frame else 'disabled'}")
        if self.passed():
            print("PROBE_RESULT PASS nested marker visible with multiple expected IDs")
        else:
            print("PROBE_RESULT FAIL nested marker not visible/detectable with enough expected IDs")


def parse_expected_ids(raw: str) -> list[int]:
    return [int(part.strip()) for part in raw.split(",") if part.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/gimbal_camera")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--expected-ids", type=parse_expected_ids, default=[10, 11, 12])
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--pass-min-ids", type=int, default=2)
    parser.add_argument("--pass-min-frames", type=int, default=3)
    parser.add_argument("--log-interval", type=float, default=2.0)
    parser.add_argument("--save-frame", default="/tmp/nested_aruco_step0_best.png")
    parser.add_argument("--save-last-frame", default="/tmp/nested_aruco_step0_last.png")
    args = parser.parse_args()

    rclpy.init()
    node = NestedArucoProbe(args)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.print_summary()
        result = 0 if node.passed() else 1
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
