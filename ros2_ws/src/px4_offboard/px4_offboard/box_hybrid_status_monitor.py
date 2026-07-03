#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dib_msgs.msg import BoxTelemetry


BOX_STATE_NAMES = {
    0: "EMPTY",
    1: "IDLE",
    2: "PREPARING_FOR_TAKEOFF",
    3: "MISSION_UPLOADING",
    4: "WAITING_FOR_TAKEOFF",
    5: "WAITING_FOR_RETURN",
    6: "PREPARING_FOR_LANDING",
    7: "WAITING_FOR_LANDING",
    8: "SECURING_DRONE",
    9: "CHARGING",
    10: "MAINTAINING",
    101: "ERROR",
}


class BoxHybridStatusMonitor(Node):
    def __init__(self) -> None:
        super().__init__("box_hybrid_status_monitor")

        self.declare_parameter("box_id", 1)
        self.declare_parameter("box_state_heartbeat_sec", 1.0)

        self.box_id = int(self.get_parameter("box_id").value)
        self.box_state_heartbeat_sec = float(self.get_parameter("box_state_heartbeat_sec").value)
        self.last_box_state = None
        self.last_box_publish_time = 0.0
        self.last_drone_state = "LANDER_UNKNOWN"
        self.last_drone_state_time = 0.0

        self.state_pub = self.create_publisher(String, "/box_hybrid_landing/state", 10)
        self.box_state_pub = self.create_publisher(
            String, "/box_hybrid_landing/box_state", 10
        )
        self.comms_pub = self.create_publisher(String, "/box_hybrid_landing/comms", 10)
        self.create_subscription(String, "/box_hybrid_landing/drone_state", self._on_drone_state, 10)
        self.create_subscription(
            BoxTelemetry, f"/b{self.box_id}/telemetry", self._on_box_telemetry, 10
        )
        self.timer = self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            f"Box hybrid status monitor ready: telemetry=/b{self.box_id}/telemetry"
        )
        self._publish(self.state_pub, self.last_drone_state)

    def _on_drone_state(self, msg: String) -> None:
        if msg.data != self.last_drone_state:
            self._publish(self.comms_pub, f"DRONE: state {self.last_drone_state} -> {msg.data}")
        self.last_drone_state = msg.data
        self.last_drone_state_time = self.get_clock().now().nanoseconds * 1e-9
        self._publish(self.state_pub, self.last_drone_state)

    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_drone_state_time and now - self.last_drone_state_time > 3.0:
            self.last_drone_state = "LANDER_STALE"
        self._publish(self.state_pub, self.last_drone_state)

    def _on_box_telemetry(self, msg: BoxTelemetry) -> None:
        state = int(msg.box_state.state)
        now = self.get_clock().now().nanoseconds * 1e-9
        heartbeat_due = (
            self.box_state_heartbeat_sec > 0.0
            and now - self.last_box_publish_time >= self.box_state_heartbeat_sec
        )

        if state == self.last_box_state and not heartbeat_due:
            return

        state_name = BOX_STATE_NAMES.get(state, f"UNKNOWN({state})")
        self._publish(self.box_state_pub, state_name)

        if state != self.last_box_state:
            prev_name = (
                "NONE"
                if self.last_box_state is None
                else BOX_STATE_NAMES.get(self.last_box_state, f"UNKNOWN({self.last_box_state})")
            )
            self._publish(self.comms_pub, f"BOX: box_state {prev_name} -> {state_name}")
            self.get_logger().info(f"box_state {prev_name} -> {state_name}")

        self.last_box_state = state
        self.last_box_publish_time = now

    @staticmethod
    def _publish(pub, text: str) -> None:
        msg = String()
        msg.data = text
        pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxHybridStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
