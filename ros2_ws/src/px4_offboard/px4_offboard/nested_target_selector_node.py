#!/usr/bin/env python3
"""Select the best nested landing target from detected camera-frame fiducials."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from dib_msgs.msg import LandingTarget6D, LandingTarget6DArray
from rclpy.node import Node


@dataclass(frozen=True)
class NestedSelectorConfig:
    outer_tag_id: int = 10
    middle_tag_id: int = 11
    inner_tag_id: int = 12
    middle_switch_in_z: float = 2.5
    middle_switch_out_z: float = 3.0
    inner_switch_in_z: float = 1.0
    inner_switch_out_z: float = 1.3
    switch_stable_sec: float = 0.5
    target_hold_sec: float = 0.5


def clone_target(target: LandingTarget6D) -> LandingTarget6D:
    return copy.deepcopy(target)


def searching_target(header) -> LandingTarget6D:
    target = LandingTarget6D()
    target.header = header
    target.state = LandingTarget6D.SEARCHING
    target.tag_id = -1
    return target


class NestedTargetSelector:
    def __init__(self, config: NestedSelectorConfig):
        self.config = config
        self.current_tag_id: Optional[int] = None
        self.candidate_tag_id: Optional[int] = None
        self.candidate_since: Optional[float] = None
        self.last_selected: Optional[LandingTarget6D] = None
        self.last_selected_time: Optional[float] = None

    def select(self, targets: Dict[int, LandingTarget6D], now: float, header) -> LandingTarget6D:
        preferred_id = self._preferred_id(targets)

        if preferred_id is None:
            held = self._held_target(now)
            if held is not None:
                held.header = header
                return held
            self.current_tag_id = None
            self.candidate_tag_id = None
            self.candidate_since = None
            return searching_target(header)

        if self.current_tag_id is None:
            return self._accept(targets[preferred_id], now, header)

        if self.current_tag_id in targets and preferred_id != self.current_tag_id:
            if self._candidate_stable(preferred_id, now):
                return self._accept(targets[preferred_id], now, header)

            current = clone_target(targets[self.current_tag_id])
            current.header = header
            self.last_selected = clone_target(current)
            self.last_selected_time = now
            return current

        if self.current_tag_id in targets:
            self.candidate_tag_id = None
            self.candidate_since = None
            return self._accept(targets[self.current_tag_id], now, header)

        held = self._held_target(now)
        if held is not None:
            held.header = header
            return held

        if self._candidate_stable(preferred_id, now):
            return self._accept(targets[preferred_id], now, header)

        return searching_target(header)

    def _preferred_id(self, targets: Dict[int, LandingTarget6D]) -> Optional[int]:
        cfg = self.config
        outer = targets.get(cfg.outer_tag_id)
        middle = targets.get(cfg.middle_tag_id)
        inner = targets.get(cfg.inner_tag_id)

        if self.current_tag_id == cfg.inner_tag_id:
            if inner is not None and inner.z <= cfg.inner_switch_out_z:
                return cfg.inner_tag_id
            if middle is not None:
                return cfg.middle_tag_id
            if outer is not None:
                return cfg.outer_tag_id

        if self.current_tag_id == cfg.middle_tag_id:
            if inner is not None and inner.z <= cfg.inner_switch_in_z:
                return cfg.inner_tag_id
            if middle is not None and middle.z <= cfg.middle_switch_out_z:
                return cfg.middle_tag_id
            if outer is not None:
                return cfg.outer_tag_id

        if inner is not None and inner.z <= cfg.inner_switch_in_z:
            return cfg.inner_tag_id
        if middle is not None and middle.z <= cfg.middle_switch_in_z:
            return cfg.middle_tag_id
        if outer is not None:
            return cfg.outer_tag_id
        if middle is not None:
            return cfg.middle_tag_id
        if inner is not None:
            return cfg.inner_tag_id
        return None

    def _candidate_stable(self, candidate_tag_id: int, now: float) -> bool:
        if self.candidate_tag_id != candidate_tag_id:
            self.candidate_tag_id = candidate_tag_id
            self.candidate_since = now
            return False

        if self.candidate_since is None:
            self.candidate_since = now
            return False

        return now - self.candidate_since >= self.config.switch_stable_sec

    def _accept(self, target: LandingTarget6D, now: float, header) -> LandingTarget6D:
        selected = clone_target(target)
        selected.header = header
        selected.state = LandingTarget6D.TRACKING
        self.current_tag_id = int(selected.tag_id)
        self.candidate_tag_id = None
        self.candidate_since = None
        self.last_selected = clone_target(selected)
        self.last_selected_time = now
        return selected

    def _held_target(self, now: float) -> Optional[LandingTarget6D]:
        if self.last_selected is None or self.last_selected_time is None:
            return None
        if now - self.last_selected_time > self.config.target_hold_sec:
            return None
        return clone_target(self.last_selected)


class NestedTargetSelectorNode(Node):
    def __init__(self):
        super().__init__("nested_target_selector_node")

        self.declare_parameter("input_topic", "/landing/targets_camera")
        self.declare_parameter("output_topic", "/landing/selected_target_camera")
        self.declare_parameter("outer_tag_id", 10)
        self.declare_parameter("middle_tag_id", 11)
        self.declare_parameter("inner_tag_id", 12)
        self.declare_parameter("middle_switch_in_z", 2.5)
        self.declare_parameter("middle_switch_out_z", 3.0)
        self.declare_parameter("inner_switch_in_z", 1.0)
        self.declare_parameter("inner_switch_out_z", 1.3)
        self.declare_parameter("switch_stable_sec", 0.5)
        self.declare_parameter("target_hold_sec", 0.5)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        cfg = NestedSelectorConfig(
            outer_tag_id=int(self.get_parameter("outer_tag_id").value),
            middle_tag_id=int(self.get_parameter("middle_tag_id").value),
            inner_tag_id=int(self.get_parameter("inner_tag_id").value),
            middle_switch_in_z=float(self.get_parameter("middle_switch_in_z").value),
            middle_switch_out_z=float(self.get_parameter("middle_switch_out_z").value),
            inner_switch_in_z=float(self.get_parameter("inner_switch_in_z").value),
            inner_switch_out_z=float(self.get_parameter("inner_switch_out_z").value),
            switch_stable_sec=float(self.get_parameter("switch_stable_sec").value),
            target_hold_sec=float(self.get_parameter("target_hold_sec").value),
        )
        if cfg.middle_switch_in_z >= cfg.middle_switch_out_z:
            raise ValueError("middle_switch_in_z must be less than middle_switch_out_z")
        if cfg.inner_switch_in_z >= cfg.inner_switch_out_z:
            raise ValueError("inner_switch_in_z must be less than inner_switch_out_z")

        self.selector = NestedTargetSelector(cfg)
        self.publisher = self.create_publisher(LandingTarget6D, output_topic, 10)
        self.create_subscription(LandingTarget6DArray, input_topic, self._on_targets, 10)

        self.get_logger().info(
            f"Nested selector ready: input={input_topic}, output={output_topic}, "
            f"ids=({cfg.outer_tag_id}, {cfg.middle_tag_id}, {cfg.inner_tag_id})"
        )

    def _on_targets(self, msg: LandingTarget6DArray):
        now = self.get_clock().now().nanoseconds / 1e9
        targets = {int(target.tag_id): target for target in msg.targets if target.state == LandingTarget6D.TRACKING}
        selected = self.selector.select(targets, now, msg.header)
        self.publisher.publish(selected)


def main(args=None):
    rclpy.init(args=args)
    node = NestedTargetSelectorNode()
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
