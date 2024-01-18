"""TouchSensorNode publishes message Button on topic /touch_sensor"""
import brickpi3
import rclpy
from rclpy.node import Node
from ev3_sensor_msgs.msg import Button


class TouchSensorNode(Node):
    """TouchSensorNode publishes message Button on topic /touch_sensor"""
    def __init__(self):
        super().__init__("touch_sensor_node")
        self.bp = brickpi3.BrickPi3()
        self.publisher = self.create_publisher(Button, "touch_sensor", 10)
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
        self.bp.set_sensor_type(self.lego_port, self.bp.SENSOR_TYPE.TOUCH)
        self.declare_parameter('frequency', 2.0)
        timer_period = 1.0/self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """reads touch sensor and publishes TouchSensor on topic /touch_sensor"""
        try:
            value = self.bp.get_sensor(self.lego_port)
            msg = Button()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "touch_sensor"
            msg.state = value
            self.publisher.publish(msg)
        except brickpi3.SensorError as e:
            error_msg = f'Invalid touch sensor data on {self.lego_port_name}'
            self.get_logger().error(error_msg)


rclpy.init()
touch_sensor_node = TouchSensorNode()
rclpy.spin(touch_sensor_node)
touch_sensor_node.destroy_node()
rclpy.shutdown()
