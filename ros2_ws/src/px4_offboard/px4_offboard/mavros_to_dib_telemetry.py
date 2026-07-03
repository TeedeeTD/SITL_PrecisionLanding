#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State as MavrosState
from mavros_msgs.msg import ExtendedState as MavrosExtendedState
from dib_msgs.msg import DroneTelemetry, State as DibState


class MavrosToDibTelemetry(Node):
    def __init__(self):
        super().__init__('mavros_to_dib_telemetry')
        self.get_logger().info("Initializing MAVROS to DIB Telemetry Bridge Node")

        # --- Parameters ---
        self.declare_parameter('drone_id', 1)
        self.drone_id = self.get_parameter('drone_id').value

        # --- Internal State Cache ---
        self.connected = False
        self.system_status = 0
        self.landed_state = DibState.LANDED_STATE_ON_GROUND  # Start on ground

        # --- Subscriptions ---
        self.mavros_state_sub = self.create_subscription(
            MavrosState,
            '/mavros/state',
            self._on_mavros_state,
            10
        )
        self.mavros_ext_state_sub = self.create_subscription(
            MavrosExtendedState,
            '/mavros/extended_state',
            self._on_mavros_extended_state,
            10
        )

        # --- Publisher ---
        self.telemetry_pub = self.create_publisher(
            DroneTelemetry,
            f'/d{self.drone_id}/telemetry',
            10
        )

        # --- Timer (10 Hz) ---
        self.timer = self.create_timer(0.1, self._on_timer)

    def _on_mavros_state(self, msg: MavrosState):
        self.connected = msg.connected
        self.system_status = msg.system_status

    def _on_mavros_extended_state(self, msg: MavrosExtendedState):
        # Mappings:
        # Mavros ExtendedState:
        # LANDED_STATE_UNDEFINED = 0
        # LANDED_STATE_ON_GROUND = 1
        # LANDED_STATE_IN_AIR = 2
        # LANDED_STATE_TAKEOFF = 3
        # LANDED_STATE_LANDING = 4
        # We can directly pass the value since our DibState constants are the same.
        self.landed_state = msg.landed_state

    def _on_timer(self):
        # Build DroneTelemetry msg
        msg = DroneTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"drone_{self.drone_id}"
        
        msg.state.connected = self.connected
        msg.state.system_status = self.system_status
        msg.state.landed_state = self.landed_state

        self.telemetry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosToDibTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
