#!/usr/bin/env python3
"""Analyze a Fractal ArUco SITL flight-test rosbag."""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TRACKING_STATE = 2
LANDED_ON_GROUND = 1
LANDED_FREEFALL = 4


def load_bag(bag_path: Path) -> dict[str, list[Any]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    data: dict[str, list[Any]] = {
        "lander_state": [],
        "setpoint": [],
        "local_pose": [],
        "target_camera": [],
        "extended_state": [],
        "mavros_state": [],
        "landing_target": [],
    }

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(raw, message_types[topic])
        t = timestamp_ns * 1e-9

        if topic == "/lander/state":
            data["lander_state"].append((t, msg.data))
        elif topic == "/mavros/setpoint_position/local":
            p = msg.pose.position
            data["setpoint"].append((t, float(p.x), float(p.y), float(p.z)))
        elif topic == "/mavros/local_position/pose":
            p = msg.pose.position
            data["local_pose"].append((t, float(p.x), float(p.y), float(p.z)))
        elif topic == "/landing/target_camera":
            data["target_camera"].append(
                (t, int(msg.state), int(msg.tag_id), float(msg.x), float(msg.y), float(msg.z))
            )
        elif topic == "/mavros/extended_state":
            data["extended_state"].append((t, int(msg.landed_state)))
        elif topic == "/mavros/state":
            data["mavros_state"].append((t, bool(msg.connected), bool(msg.armed), msg.mode))
        elif topic == "/mavros/landing_target/raw":
            data["landing_target"].append((t, float(msg.distance)))

    return data


def transitions(states: list[tuple[float, str]]) -> list[tuple[float, str | None, str]]:
    result: list[tuple[float, str | None, str]] = []
    previous: str | None = None
    for t, state in states:
        if state != previous:
            result.append((t, previous, state))
            previous = state
    return result


def windows_from_transitions(
    trans: list[tuple[float, str | None, str]],
    final_time: float,
    state_name: str,
) -> list[tuple[float, float]]:
    windows: list[tuple[float, float]] = []
    for idx, (start, _, state) in enumerate(trans):
        end = trans[idx + 1][0] if idx + 1 < len(trans) else final_time
        if state == state_name:
            windows.append((start, end))
    return windows


def in_windows(timestamp: float, windows: list[tuple[float, float]]) -> bool:
    return any(start <= timestamp <= end for start, end in windows)


def rate(samples: list[tuple[Any, ...]]) -> float:
    if len(samples) < 2:
        return 0.0
    duration = float(samples[-1][0] - samples[0][0])
    if duration <= 0.0:
        return 0.0
    return (len(samples) - 1) / duration


def format_transition_sequence(trans: list[tuple[float, str | None, str]]) -> str:
    return " -> ".join(state for _, _, state in trans)


def analyze(data: dict[str, list[Any]]) -> dict[str, Any]:
    states = data["lander_state"]
    trans = transitions(states)
    final_time = max((samples[-1][0] for samples in data.values() if samples), default=0.0)
    descent_windows = windows_from_transitions(trans, final_time, "DESCEND_OVER_TARGET")
    offboard_expected_windows: list[tuple[float, float]] = []
    for state_name in (
        "FLY_TO_SEARCH",
        "SEARCH",
        "HORIZONTAL_APPROACH",
        "DESCEND_OVER_TARGET",
        "TARGET_LOST",
    ):
        offboard_expected_windows.extend(windows_from_transitions(trans, final_time, state_name))

    setpoints = data["setpoint"]
    poses = data["local_pose"]
    targets = data["target_camera"]
    extended = data["extended_state"]
    mavros = data["mavros_state"]
    landing_targets = data["landing_target"]

    setpoints_desc = [sample for sample in setpoints if in_windows(sample[0], descent_windows)]
    poses_desc = [sample for sample in poses if in_windows(sample[0], descent_windows)]
    targets_desc = [sample for sample in targets if in_windows(sample[0], descent_windows)]

    z_jumps = []
    for before, after in zip(setpoints_desc, setpoints_desc[1:]):
        dz = after[3] - before[3]
        if dz > 0.08:
            z_jumps.append((before[0], after[0], before[3], after[3], dz))

    state_names = [state for _, state in states]
    required_sequence = ["SEARCH", "HORIZONTAL_APPROACH", "DESCEND_OVER_TARGET", "LAND", "DONE"]
    sequence_cursor = 0
    for _, _, state in trans:
        if sequence_cursor < len(required_sequence) and state == required_sequence[sequence_cursor]:
            sequence_cursor += 1
    has_required_sequence = sequence_cursor == len(required_sequence)

    bad_reset_count = 0
    for idx, (_, old_state, new_state) in enumerate(trans):
        if old_state == "TARGET_LOST" and new_state == "SEARCH":
            bad_reset_count += 1
        if old_state == "DESCEND_OVER_TARGET" and new_state == "SEARCH":
            bad_reset_count += 1

    final_pose = poses[-1] if poses else None
    final_setpoint = setpoints[-1] if setpoints else None
    final_extended = extended[-1] if extended else None
    final_mavros = mavros[-1] if mavros else None

    landed_seen = any(state == LANDED_ON_GROUND for _, state in extended)
    disarmed_final = final_mavros is not None and not final_mavros[2]
    offboard_drop = any(
        mode != "OFFBOARD"
        for timestamp, connected, armed, mode in mavros
        if armed and in_windows(timestamp, offboard_expected_windows)
    )
    mavros_disconnect = any(not connected for _, connected, _, _ in mavros)

    target_state_counts = Counter(state for _, state, *_ in targets)
    target_desc_counts = Counter(state for _, state, *_ in targets_desc)
    tracking_desc = target_desc_counts.get(TRACKING_STATE, 0)
    target_desc_total = sum(target_desc_counts.values())
    tracking_ratio_desc = tracking_desc / target_desc_total if target_desc_total else 0.0

    descent_error_samples = []
    for pose in poses_desc:
        _, x, y, _z = pose
        # SITL pad/search target is configured at ENU (3.0, 2.0).
        descent_error_samples.append(math.hypot(x - 3.0, y - 2.0))
    max_descent_xy_error = max(descent_error_samples) if descent_error_samples else float("nan")
    final_xy_error = (
        math.hypot(final_pose[1] - 3.0, final_pose[2] - 2.0)
        if final_pose is not None
        else float("nan")
    )

    checks = {
        "fsm_sequence": has_required_sequence,
        "no_descent_reset_to_search": bad_reset_count == 0,
        "setpoint_rate": rate(setpoints) >= 20.0,
        "target_rate": rate(targets) >= 20.0,
        "descent_z_monotonic": len(z_jumps) == 0 and bool(setpoints_desc),
        "landed_seen": landed_seen,
        "final_disarmed": disarmed_final,
        "mavros_connected": not mavros_disconnect,
        "offboard_stable_while_armed": not offboard_drop,
        "landing_target_published": len(landing_targets) > 0,
    }

    return {
        "transitions": trans,
        "state_counts": Counter(state_names),
        "checks": checks,
        "overall_pass": all(checks.values()),
        "setpoint_rate": rate(setpoints),
        "target_rate": rate(targets),
        "landing_target_rate": rate(landing_targets),
        "descent_windows": descent_windows,
        "descent_sp_start": setpoints_desc[0][3] if setpoints_desc else float("nan"),
        "descent_sp_end": setpoints_desc[-1][3] if setpoints_desc else float("nan"),
        "descent_sp_min": min((sp[3] for sp in setpoints_desc), default=float("nan")),
        "descent_sp_max": max((sp[3] for sp in setpoints_desc), default=float("nan")),
        "descent_pose_start": poses_desc[0][3] if poses_desc else float("nan"),
        "descent_pose_end": poses_desc[-1][3] if poses_desc else float("nan"),
        "descent_pose_min": min((p[3] for p in poses_desc), default=float("nan")),
        "descent_pose_max": max((p[3] for p in poses_desc), default=float("nan")),
        "z_jumps": z_jumps,
        "bad_reset_count": bad_reset_count,
        "target_state_counts": target_state_counts,
        "target_desc_counts": target_desc_counts,
        "tracking_ratio_desc": tracking_ratio_desc,
        "max_descent_xy_error": max_descent_xy_error,
        "final_xy_error": final_xy_error,
        "final_pose": final_pose,
        "final_setpoint": final_setpoint,
        "final_extended": final_extended,
        "final_mavros": final_mavros,
    }


def print_report(bag_path: Path, result: dict[str, Any]) -> None:
    status = "PASS" if result["overall_pass"] else "FAIL"
    print(f"Fractal flight test: {status}")
    print(f"Bag: {bag_path}")
    print()

    print("FSM transitions:")
    for timestamp, old_state, new_state in result["transitions"]:
        print(f"  {timestamp:.3f}: {old_state or '<start>'} -> {new_state}")
    print()

    print("Metrics:")
    print(f"  setpoint_rate_hz:       {result['setpoint_rate']:.2f}")
    print(f"  target_camera_rate_hz:  {result['target_rate']:.2f}")
    print(f"  landing_target_rate_hz: {result['landing_target_rate']:.2f}")
    print(
        "  descent_setpoint_z:    "
        f"{result['descent_sp_start']:.2f} -> {result['descent_sp_end']:.2f} m "
        f"(min={result['descent_sp_min']:.2f}, max={result['descent_sp_max']:.2f})"
    )
    print(
        "  descent_vehicle_z:     "
        f"{result['descent_pose_start']:.2f} -> {result['descent_pose_end']:.2f} m "
        f"(min={result['descent_pose_min']:.2f}, max={result['descent_pose_max']:.2f})"
    )
    print(f"  descent_z_large_jumps:  {len(result['z_jumps'])}")
    print(f"  target_tracking_ratio_during_descent: {100.0 * result['tracking_ratio_desc']:.1f}%")
    print(f"  max_descent_xy_error_to_pad: {result['max_descent_xy_error']:.2f} m")
    print(f"  final_xy_error_to_pad:       {result['final_xy_error']:.2f} m")
    print(f"  target_state_counts:    {dict(result['target_state_counts'])}")
    print(f"  target_desc_counts:     {dict(result['target_desc_counts'])}")
    print(f"  final_pose:             {result['final_pose']}")
    print(f"  final_setpoint:         {result['final_setpoint']}")
    print(f"  final_extended_state:   {result['final_extended']}")
    print(f"  final_mavros_state:     {result['final_mavros']}")
    print()

    print("Checks:")
    for name, passed in result["checks"].items():
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
    print()

    print("Requirement evidence:")
    print("  REQ_PL_CTRL_001: PASS if setpoint_rate_hz >= 20.")
    print("  REQ_PL_OUT_001:  PASS if landing target is published and PX4 reaches LAND/DONE.")
    print("  REQ_PL_DET_002:  SITL evidence if tracking exists through high-to-low descent.")
    print("  REQ_PL_DET_001:  Needs multi-run/statistical detection-rate evidence for >95%.")
    print("  REQ_PL_VIDEO_001: Needs explicit latency/image timestamp evidence for <100 ms.")
    print("  REQ_PL_COORD_001/002: Needs static calibration and 4-direction sign test.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Path to a rosbag2 directory")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.bag.exists():
        print(f"Bag does not exist: {args.bag}", file=sys.stderr)
        return 2

    result = analyze(load_bag(args.bag))
    print_report(args.bag, result)
    return 0 if result["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
