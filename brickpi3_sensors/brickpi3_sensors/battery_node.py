"""Battery Sensor Node"""
import brickpi3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryNode(Node):
    """Publishes BatteryState message on /battery_state"""
    def __init__(self):
        super().__init__("battery_node")
        self.bp = brickpi3.BrickPi3()
        self.publisher = self.create_publisher(BatteryState, "/battery_state", 10)
        self.declare_parameter('frequency', 1.0/60.0)
        timer_period = 1.0/self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_callback()

    def timer_callback(self):
        """Reads and publishes BatteryState message on battery voltage on /battery_state"""
        battery_voltage = self.bp.get_voltage_battery()
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "battery_sensor"
        msg.voltage = battery_voltage
        self.publisher.publish(msg)


rclpy.init()
battery_node = BatteryNode()
rclpy.spin(battery_node)
battery_node.destroy_node()
rclpy.shutdown()
