#!/usr/bin/env python3
"""Generate a centered multi-ID ArUco marker for nested-selector tests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np


DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "docs" / "nested_aruco_4x4_50.png"
DEFAULT_METADATA = Path(__file__).resolve().parents[1] / "docs" / "nested_aruco_4x4_50.json"


def dictionary_constant(name: str) -> int:
    if hasattr(aruco, name):
        return int(getattr(aruco, name))

    matches = [candidate for candidate in dir(aruco) if candidate.lower() == name.lower()]
    if matches:
        return int(getattr(aruco, matches[0]))

    raise ValueError(f"OpenCV aruco dictionary not found: {name}")


def aruco_dictionary(name: str):
    dictionary_id = dictionary_constant(name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


def generate_marker(dictionary, marker_id: int, marker_px: int):
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(dictionary, marker_id, marker_px)

    marker = np.zeros((marker_px, marker_px), dtype="uint8")
    aruco.drawMarker(dictionary, marker_id, marker_px, marker, 1)
    return marker


def marker_image(dictionary, marker_id: int, marker_px: int, quiet_px: int):
    marker = generate_marker(dictionary, marker_id, marker_px)
    if quiet_px <= 0:
        return marker
    return cv2.copyMakeBorder(
        marker,
        quiet_px,
        quiet_px,
        quiet_px,
        quiet_px,
        cv2.BORDER_CONSTANT,
        value=255,
    )


def paste_center(canvas, image):
    height, width = image.shape[:2]
    y0 = (canvas.shape[0] - height) // 2
    x0 = (canvas.shape[1] - width) // 2
    canvas[y0 : y0 + height, x0 : x0 + width] = image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--metadata", type=Path, default=DEFAULT_METADATA)
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--outer-id", type=int, default=10)
    parser.add_argument("--middle-id", type=int, default=11)
    parser.add_argument("--inner-id", type=int, default=12)
    parser.add_argument("--canvas-px", type=int, default=2400)
    parser.add_argument("--outer-ratio", type=float, default=0.80)
    parser.add_argument("--middle-ratio", type=float, default=0.22)
    parser.add_argument("--inner-ratio", type=float, default=0.08)
    parser.add_argument("--quiet-ratio", type=float, default=0.15)
    parser.add_argument("--outer-size-m", type=float, default=1.0)
    args = parser.parse_args()

    dictionary = aruco_dictionary(args.dictionary)

    canvas = 255 * np.ones((args.canvas_px, args.canvas_px), dtype="uint8")
    outer_px = int(args.canvas_px * args.outer_ratio)
    middle_px = int(outer_px * args.middle_ratio)
    inner_px = int(outer_px * args.inner_ratio)

    paste_center(canvas, marker_image(dictionary, args.outer_id, outer_px, 0))
    paste_center(
        canvas,
        marker_image(dictionary, args.middle_id, middle_px, int(middle_px * args.quiet_ratio)),
    )
    paste_center(
        canvas,
        marker_image(dictionary, args.inner_id, inner_px, int(inner_px * args.quiet_ratio)),
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), canvas):
        raise RuntimeError(f"failed to write {args.output}")

    metadata = {
        "dictionary": args.dictionary,
        "layout": "centered_overlay",
        "output": str(args.output),
        "canvas_px": args.canvas_px,
        "outer": {"id": args.outer_id, "size_m": args.outer_size_m, "marker_px": outer_px},
        "middle": {
            "id": args.middle_id,
            "size_m": args.outer_size_m * args.middle_ratio,
            "marker_px": middle_px,
        },
        "inner": {
            "id": args.inner_id,
            "size_m": args.outer_size_m * args.inner_ratio,
            "marker_px": inner_px,
        },
        "center_offset_m": {"outer": [0.0, 0.0], "middle": [0.0, 0.0], "inner": [0.0, 0.0]},
        "note": "All marker centers are aligned with the landing point. The flight selector must use per-ID marker sizes.",
    }
    args.metadata.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    print(f"wrote {args.output}")
    print(f"wrote {args.metadata}")
    print(f"ids: outer={args.outer_id}, middle={args.middle_id}, inner={args.inner_id}")
    print(f"sizes_m: outer={args.outer_size_m:.3f}, middle={metadata['middle']['size_m']:.3f}, inner={metadata['inner']['size_m']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
