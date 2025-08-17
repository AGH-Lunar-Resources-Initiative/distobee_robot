import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

LEFT_X = 0
LEFT_Y = 1
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5

MODE = False  # Default manual driving | TO DO: add tempomat
WHEEL_BASE = 0.7 # 

class GamepadDrive(Node):
    def __init__(self):
        super().__init__("GamepadDrive")

        self.max_steering_angle = self.declare_parameter(
            "max_steering_angle", np.deg2rad(30)).value  # to calculate turning radius
        self.max_speed = self.declare_parameter(
            "max_speed", 0.15).value  #  15 cm/s experimental

        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_cb, qos_profile=10)

        self.vel_pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10)

    # Triggers as linear speed, LEFT_X as angular
    def joy_cb(self, msg: Joy):
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"

        speed = self.max_speed * ((-msg.axes[RIGHT_TRIGGER] * 0.5 + 0.5) - (-msg.axes[LEFT_TRIGGER] * 0.5 + 0.5))
        phi = self.max_steering_angle * msg.axes[LEFT_X]

        if abs(speed) < 1e-3:  # To prevent noisy input caused by joys deadzone
            vel_msg.twist.angular.z = 0
            vel_msg.twist.linear.x = 0
        else:
            vel_msg.twist.linear.x = speed
            vel_msg.twist.angular.z = (speed * np.tan(phi)) / WHEEL_BASE

        self.vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = GamepadDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
