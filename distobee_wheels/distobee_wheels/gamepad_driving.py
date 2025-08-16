import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


LEFT_X = 0
LEFT_Y = 1
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5

MODE = False  # Default manual driving TO DO: add tempomat


class GamepadDrive(Node):
    def __init__(self):
        super().__init__("GamepadDrive")

        self.max_steering_angle = self.declare_parameter(
            "max_steering_angle", 0.5).value  # in rad
        self.max_speed = self.declare_parameter(
            "max_speed", 0.15).value  # in m/s

        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_cb, qos_profile=10)

        self.vel_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_steering_controller/reference', 10)

    # Triggers as linear speed, LEFT_X as angular
    def joy_cb(self, msg: Joy):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = "base_link"

        ack_msg.drive.speed = self.max_speed * ((-msg.axes[RIGHT_TRIGGER] * 0.5 + 0.5) - (-msg.axes[LEFT_TRIGGER] * 0.5 + 0.5))
        ack_msg.drive.steering_angle = self.max_steering_angle * msg.axes[LEFT_X]

        self.vel_pub.publish(ack_msg)


def main():
    rclpy.init()
    node = GamepadDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
