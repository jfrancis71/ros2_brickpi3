"""Ultrasonic Distance Sensor Node"""
import brickpi3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UltrasonicDistanceNode(Node):
    """Publishes Range message on topic ultrasonic_distance"""
    def __init__(self):
        super().__init__("ultrasonic_distance_node")
        self.bp = brickpi3.BrickPi3()
        self.publisher = self.create_publisher(Range, "ultrasonic_distance", 10)
        self.declare_parameter('lego_port', 'PORT_1')
        port_dict = { "PORT_1": self.bp.PORT_1,
              "PORT_2": self.bp.PORT_2,
              "PORT_3": self.bp.PORT_3,
              "PORT_4": self.bp.PORT_4 }
        self.lego_port_name = self.get_parameter('lego_port').get_parameter_value().string_value
        try:
            self.lego_port = port_dict[self.lego_port_name]
        except KeyError as e:
            error_msg = f'Unknown lego input port: {e}'
            self.get_logger().fatal(error_msg)
            raise IOError(error_msg) from e
        self.lego_port = port_dict[self.lego_port_name]
        # we disable pylint warning as BrickPi does some strange attribute manipulation
        self.bp.set_sensor_type(self.lego_port, self.bp.SENSOR_TYPE.EV3_ULTRASONIC_CM)  # pylint: disable=E1101
        self.declare_parameter('frequency', 2.0)
        timer_period = 1.0/self.get_parameter('frequency').get_parameter_value().double_value

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Reads ultrasonic distance and publishes Range message on topic ultrasonic_distance"""
        try:
            ultrasonic_distance = self.bp.get_sensor(self.lego_port)
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ultrasonic_distance_sensor"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.05  # very approximate
            msg.min_range = 0.0
            msg.max_range = 1.0
            msg.range = ultrasonic_distance/100.0  # raw sensor is in cm's
            self.publisher.publish(msg)
        except brickpi3.SensorError as e:
            error_msg = f'Invalid ultrasonic distance sensor data on {self.lego_port_name}'
            self.get_logger().error(error_msg)


rclpy.init()
ultrasonic_distance_node = UltrasonicDistanceNode()
rclpy.spin(ultrasonic_distance_node)
ultrasonic_distance_node.destroy_node()
rclpy.shutdown()
