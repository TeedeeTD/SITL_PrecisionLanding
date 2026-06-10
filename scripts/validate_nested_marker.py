#!/usr/bin/env python3
"""Validate whether a candidate nested ArUco marker exposes multiple IDs.

The Step 0 acceptance gate is intentionally strict:
at least two IDs must be detected stably across the tested views/scales.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

import cv2
import cv2.aruco as aruco


DEFAULT_MARKER = Path(__file__).resolve().parents[1] / "docs" / "marker.png"
DEFAULT_DICT = "DICT_ARUCO_MIP_36h12"


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


def detector_for(dictionary_name: str) -> ArucoDetectorCompat:
    params = detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.minMarkerPerimeterRate = 0.001
    params.maxMarkerPerimeterRate = 4.0
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 153
    params.adaptiveThreshWinSizeStep = 10
    return ArucoDetectorCompat(aruco_dictionary(dictionary_name), params)


def detect_ids(detector: ArucoDetectorCompat, gray) -> tuple[list[int], int]:
    _, ids, rejected = detector.detectMarkers(gray)
    if ids is None:
        return [], len(rejected)
    return ids.flatten().astype(int).tolist(), len(rejected)


def centered_crop(gray, ratio: float):
    height, width = gray.shape[:2]
    size = int(min(height, width) * ratio)
    x0 = width // 2 - size // 2
    y0 = height // 2 - size // 2
    return gray[y0 : y0 + size, x0 : x0 + size]


def iter_views(gray) -> Iterable[tuple[str, object]]:
    for scale in (0.10, 0.075, 0.05):
        resized = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        yield f"full scale={scale:g}", resized

    for ratio in (0.70, 0.50, 0.35, 0.25, 0.18, 0.12, 0.08):
        crop = centered_crop(gray, ratio)
        resized = cv2.resize(crop, (900, 900), interpolation=cv2.INTER_AREA)
        yield f"center crop={ratio:g} resized=900", resized


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, default=DEFAULT_MARKER)
    parser.add_argument("--dictionary", default=DEFAULT_DICT)
    parser.add_argument(
        "--stable-count",
        type=int,
        default=2,
        help="Minimum number of tested views where an ID must appear to be considered stable.",
    )
    args = parser.parse_args()

    gray = cv2.imread(str(args.image), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(args.image)

    detector = detector_for(args.dictionary)
    id_counts: Counter[int] = Counter()
    id_views: dict[int, list[str]] = defaultdict(list)

    print(f"image: {args.image}")
    print(f"shape: {gray.shape[1]}x{gray.shape[0]}")
    print(f"dictionary: {args.dictionary}")
    print()

    for label, view in iter_views(gray):
        ids, rejected = detect_ids(detector, view)
        print(f"{label}: ids={ids if ids else 'none'} rejected={rejected}")
        for marker_id in ids:
            id_counts[marker_id] += 1
            id_views[marker_id].append(label)

    stable_ids = sorted(marker_id for marker_id, count in id_counts.items() if count >= args.stable_count)

    print()
    print(f"unique_ids: {sorted(id_counts)}")
    print(f"stable_ids_count>={args.stable_count}: {stable_ids}")
    for marker_id in sorted(id_views):
        print(f"id {marker_id}: seen {id_counts[marker_id]} time(s) in {id_views[marker_id]}")

    if len(stable_ids) >= 2:
        print()
        print("RESULT: PASS - nested multi-ID detection is stable enough for selector testing.")
        return 0

    print()
    print("RESULT: FAIL - fewer than two stable marker IDs were detected.")
    print("ACTION: choose or generate a corrected nested marker with separate valid IDs.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
