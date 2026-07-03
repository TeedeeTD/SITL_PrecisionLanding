#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from dib_msgs.msg import (
    LidStatus,
    ClampStatus,
    ChargeStatus,
    CoolingStatus,
    PowerStatus,
    RTKInfo,
    WindSensor,
    Temperature,
    Humidity,
    Rain
)
from dib_msgs.srv import (
    LidCmd,
    ClampCmd,
    ChargeCmd,
    CoolingCmd,
    PowerButtonCmd
)
from sensor_msgs.msg import NavSatFix


class MockBoxHardware(Node):
    def __init__(self):
        super().__init__('mock_box_hardware')
        self.get_logger().info("Initializing Mock Box Hardware Node")

        # --- Parameters ---
        self.declare_parameter('box_id', 1)
        self.declare_parameter('pos_clamp_h_close', 8850)
        self.declare_parameter('pos_clamp_v_close', 4100)
        self.box_id = self.get_parameter('box_id').value
        self.pos_clamp_h_close = self.get_parameter('pos_clamp_h_close').value
        self.pos_clamp_v_close = self.get_parameter('pos_clamp_v_close').value

        # --- Internal States ---
        # Lid state: CLOSED=0, OPENED=1, CLOSING=2, OPENING=3
        self.lid_state = LidStatus.CLOSED
        self.lid_target_state = LidStatus.CLOSED
        self.lid_transition_time = 0.0

        # Clamp state: positions
        self.clamp_h_pos = 0
        self.clamp_v_pos = 0
        self.clamp_h_target = 0
        self.clamp_v_target = 0
        self.clamp_speed = 2000  # units per second

        # Charge state: NOT_CHARGING=0, CHARGING=1, CHARGE_DONE=2
        self.charge_state = ChargeStatus.NOT_CHARGING
        self.charge_fault_status = 0
        self.v_bat = 0.0
        self.i_bat = 0.0

        # Cooling state
        self.cooling_state = 0

        # Power state
        self.power_battery_voltage = 24.2

        # --- Create Service Servers ---
        self.lid_srv = self.create_service(LidCmd, '/lid/cmd', self._handle_lid_cmd)
        self.clamp_srv = self.create_service(ClampCmd, '/clamp/cmd', self._handle_clamp_cmd)
        self.charge_srv = self.create_service(ChargeCmd, '/dock/charge/cmd', self._handle_charge_cmd)
        self.cooling_srv = self.create_service(CoolingCmd, '/dock/cooling_battery/cmd', self._handle_cooling_cmd)
        self.power_srv = self.create_service(PowerButtonCmd, '/dock/power_button/cmd', self._handle_power_cmd)

        # --- Create Publishers ---
        self.lid_pub = self.create_publisher(LidStatus, '/lid/status', 10)
        self.clamp_pub = self.create_publisher(ClampStatus, '/clamp/status', 10)
        self.charge_pub = self.create_publisher(ChargeStatus, '/dock/charge/status', 10)
        self.cooling_pub = self.create_publisher(CoolingStatus, '/dock/cooling_battery/status', 10)
        self.power_pub = self.create_publisher(PowerStatus, '/system1/power/status', 10)
        self.battery_temp_pub = self.create_publisher(Temperature, '/dock/battery/temperature', 10)

        # Environment Publishers
        self.wind_pub = self.create_publisher(WindSensor, 'env/outside/wind', 10)
        self.temp_inside_pub = self.create_publisher(Temperature, 'system1/temperature2', 10)
        self.temp_outside_pub = self.create_publisher(Temperature, 'env/outside/temperature', 10)
        self.humidity_inside_pub = self.create_publisher(Humidity, 'system1/humidity1', 10)
        self.humidity_outside_pub = self.create_publisher(Humidity, 'env/outside/humidity', 10)
        self.rain_pub = self.create_publisher(Rain, 'env/outside/rain', 10)

        # GPS / RTK Publishers
        self.gps_pub = self.create_publisher(NavSatFix, 'gps', 10)
        self.rtk_pub = self.create_publisher(RTKInfo, 'rtk_info', 10)

        # --- Tick Timer (10 Hz) ---
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self._on_timer)

    # --- Service Callbacks ---
    def _handle_lid_cmd(self, request, response):
        cmd = request.command
        self.get_logger().info(f"Lid Command Received: {cmd}")
        if cmd == 1:  # Open
            if self.lid_state == LidStatus.CLOSED:
                self.lid_state = LidStatus.OPENING
                self.lid_target_state = LidStatus.OPENED
                self.lid_transition_time = time.time()
        elif cmd == 0:  # Close
            if self.lid_state == LidStatus.OPENED:
                self.lid_state = LidStatus.CLOSING
                self.lid_target_state = LidStatus.CLOSED
                self.lid_transition_time = time.time()
        response.success = True
        return response

    def _handle_clamp_cmd(self, request, response):
        mode = request.mode
        clamp_h_pos_cmd = request.clamp_h_pos_cmd
        clamp_v_pos_cmd = request.clamp_v_pos_cmd
        clamp_select = request.clamp_select
        self.get_logger().info(f"Clamp Command: select={clamp_select}, mode={mode}, h={clamp_h_pos_cmd}, v={clamp_v_pos_cmd}")

        if clamp_select == 1:  # H Clamp
            self.clamp_h_target = clamp_h_pos_cmd
        elif clamp_select == 2:  # V Clamp
            self.clamp_v_target = clamp_v_pos_cmd
        elif clamp_select == 3:  # Release/Open all
            self.clamp_h_target = 0
            self.clamp_v_target = 0
        response.success = True
        return response

    def _handle_charge_cmd(self, request, response):
        cmd = request.command
        self.get_logger().info(f"Charge Command Received: {cmd}")
        if cmd == 1:
            self.charge_state = ChargeStatus.CHARGING
            self.v_bat = 24.8
            self.i_bat = 2.5
        elif cmd == 0:
            self.charge_state = ChargeStatus.NOT_CHARGING
            self.v_bat = 0.0
            self.i_bat = 0.0
        response.success = True
        return response

    def _handle_cooling_cmd(self, request, response):
        cmd = request.command
        self.get_logger().info(f"Cooling Command Received: {cmd}")
        self.cooling_state = cmd
        response.success = True
        return response

    def _handle_power_cmd(self, request, response):
        cmd = request.command
        self.get_logger().info(f"Power Button Command Received: {cmd}")
        # Transition battery voltage based on power
        if cmd == 1:
            self.power_battery_voltage = 24.2
        else:
            self.power_battery_voltage = 0.0
        response.success = True
        return response

    # --- Timer Loop ---
    def _on_timer(self):
        now_ts = time.time()

        # Update Lid state transitions
        if self.lid_state in [LidStatus.OPENING, LidStatus.CLOSING]:
            if now_ts - self.lid_transition_time >= 2.0:
                self.lid_state = self.lid_target_state
                self.get_logger().info(f"Lid transition completed: {self.lid_state}")

        # Update Clamp positions step-by-step
        step = self.clamp_speed * self.timer_period
        # Horizontal clamp
        if self.clamp_h_pos < self.clamp_h_target:
            self.clamp_h_pos = min(self.clamp_h_target, int(self.clamp_h_pos + step))
        elif self.clamp_h_pos > self.clamp_h_target:
            self.clamp_h_pos = max(self.clamp_h_target, int(self.clamp_h_pos - step))
        # Vertical clamp
        if self.clamp_v_pos < self.clamp_v_target:
            self.clamp_v_pos = min(self.clamp_v_target, int(self.clamp_v_pos + step))
        elif self.clamp_v_pos > self.clamp_v_target:
            self.clamp_v_pos = max(self.clamp_v_target, int(self.clamp_v_pos - step))

        # Charge progress simulator
        if self.charge_state == ChargeStatus.CHARGING:
            # Simulate battery voltage slowly rising
            self.v_bat = min(25.2, self.v_bat + 0.005)
            if self.v_bat >= 25.2:
                self.charge_state = ChargeStatus.CHARGE_DONE
                self.i_bat = 0.0
        elif self.charge_state == ChargeStatus.CHARGE_DONE:
            self.v_bat = 25.2
            self.i_bat = 0.0

        # --- Publish Status Messages ---
        # Lid Status
        lid_msg = LidStatus()
        lid_msg.lid_status = self.lid_state
        self.lid_pub.publish(lid_msg)

        # Clamp Status
        clamp_msg = ClampStatus()
        clamp_msg.clamp_h_pos = self.clamp_h_pos
        clamp_msg.clamp_v_pos = self.clamp_v_pos
        self.clamp_pub.publish(clamp_msg)

        # Charge Status
        charge_msg = ChargeStatus()
        charge_msg.charge_status = self.charge_state
        charge_msg.fault_status = self.charge_fault_status
        charge_msg.v_bat = self.v_bat
        charge_msg.i_bat = self.i_bat
        self.charge_pub.publish(charge_msg)

        # Cooling Status
        cooling_msg = CoolingStatus()
        cooling_msg.cooling_status = self.cooling_state
        self.cooling_pub.publish(cooling_msg)

        # Power Status
        power_msg = PowerStatus()
        power_msg.battery_voltage = self.power_battery_voltage
        self.power_pub.publish(power_msg)

        # Battery temperature
        battery_temp_msg = Temperature()
        battery_temp_msg.temperature = 28.5 if self.cooling_state == 0 else 24.0
        self.battery_temp_pub.publish(battery_temp_msg)

        # Environment
        wind_msg = WindSensor()
        wind_msg.wind_speed = 2.4
        wind_msg.wind_direction = 120.0
        self.wind_pub.publish(wind_msg)

        temp_inside_msg = Temperature()
        temp_inside_msg.temperature = 31.0
        self.temp_inside_pub.publish(temp_inside_msg)

        temp_outside_msg = Temperature()
        temp_outside_msg.temperature = 29.5
        self.temp_outside_pub.publish(temp_outside_msg)

        hum_inside_msg = Humidity()
        hum_inside_msg.humidity = 45.0
        self.humidity_inside_pub.publish(hum_inside_msg)

        hum_outside_msg = Humidity()
        hum_outside_msg.humidity = 50.0
        self.humidity_outside_pub.publish(hum_outside_msg)

        rain_msg = Rain()
        rain_msg.rain = False
        self.rain_pub.publish(rain_msg)

        # --- GPS / RTK Coordinates ---
        # Default home of PX4 SITL is at Zurich airport: (47.397742, 8.545594)
        # Gazebo coordinates of the box center: (4.0, -3.5)
        # Converting using rough local offsets:
        # lat offset: 4.0 meters North -> 4.0 * 9.0e-6 = 3.6e-5
        # lon offset: -3.5 meters East -> -3.5 * 1.33e-5 = -4.655e-5
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_box"
        gps_msg.latitude = 47.397742 + 3.6e-5
        gps_msg.longitude = 8.545594 - 4.655e-5
        gps_msg.altitude = 488.0
        self.gps_pub.publish(gps_msg)

        rtk_msg = RTKInfo()
        rtk_msg.gps_rtk_status_fix = 2  # Fixed RTK
        rtk_msg.num_sat = 16
        self.rtk_pub.publish(rtk_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockBoxHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
