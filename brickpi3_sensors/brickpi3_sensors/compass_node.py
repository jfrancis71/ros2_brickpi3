"""CompassNode publishes message UInt16 on topic compass_bearing"""
import time
import math
from scipy.spatial.transform import Rotation as R
import brickpi3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped


class CompassNode(Node):
    """CompassNode publishes message QuaternionStamped on topic compass_bearing"""
    def __init__(self):
        super().__init__("compass_node")
        self.bp = brickpi3.BrickPi3()
        self.publisher = self.create_publisher(QuaternionStamped, "compass_bearing", 10)
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
        # Ref 1, also we disable pylint warning as BrickPi does some strange attribute manipulation
        self.bp.set_sensor_type(self.lego_port, self.bp.SENSOR_TYPE.I2C, [0,20]) # pylint: disable=E1101
        self.declare_parameter('frequency', 2.0)
        timer_period = 1.0/self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """reads compass bearing and publishes on topic compass_bearing"""
        self.bp.transact_i2c(self.lego_port, 0b00000010, [0x42], 2) # Ref 1
        time.sleep(.01)
        try:
            value = self.bp.get_sensor(self.lego_port)
            compass_bearing = value[0]*2 + value[1]
            compass_bearing_rad = compass_bearing*2.0*math.pi/360.0
            quaternion = R.from_euler('xyz',[0, 0, -compass_bearing_rad]).as_quat()
            msg = QuaternionStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "compass_sensor"
            q = msg.quaternion
            q.x, q.y, q.z, q.w = quaternion
            self.publisher.publish(msg)
        except brickpi3.SensorError as e:
            error_msg = f'Invalid compass sensor data on {self.lego_port_name}'
            self.get_logger().error(error_msg)


rclpy.init()
compass_node = CompassNode()
rclpy.spin(compass_node)
compass_node.destroy_node()
rclpy.shutdown()


# Ref 1: https://forum.dexterindustries.com/t/hitechnic-compass-sensor-with-brickpi3/7906
