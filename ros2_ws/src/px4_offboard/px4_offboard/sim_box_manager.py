"""Lightweight box-state simulator for hybrid landing SITL tests."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimBoxManager(Node):
    def __init__(self) -> None:
        super().__init__("sim_box_manager")

        self.declare_parameter("state_topic", "/sim_box/state")
        self.declare_parameter("prepare_after_sec", 5.0)
        self.declare_parameter("ready_after_sec", 8.0)
        self.declare_parameter("publish_hz", 5.0)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.prepare_after_sec = float(self.get_parameter("prepare_after_sec").value)
        self.ready_after_sec = float(self.get_parameter("ready_after_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.started_at = self._now()
        self.state = "IDLE"
        self.pub = self.create_publisher(String, self.state_topic, 10)
        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            "Sim box manager ready: "
            f"state_topic={self.state_topic} ready_after_sec={self.ready_after_sec}"
        )

    def _tick(self) -> None:
        elapsed = self._now() - self.started_at
        if elapsed >= self.ready_after_sec:
            self._set_state("WAITING_FOR_LANDING")
        elif elapsed >= self.prepare_after_sec:
            self._set_state("PREPARING_FOR_LANDING")

        msg = String()
        msg.data = self.state
        self.pub.publish(msg)

    def _set_state(self, new_state: str) -> None:
        if self.state == new_state:
            return
        self.get_logger().info(f"Box state: {self.state} -> {new_state}")
        self.state = new_state

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBoxManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
