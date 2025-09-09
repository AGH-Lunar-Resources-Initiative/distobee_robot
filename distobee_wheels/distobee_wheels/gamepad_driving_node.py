import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

# Gamepad axis indices
LEFT_X = 0
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5

class GamepadDriving(Node):
    def __init__(self):
        super().__init__("gamepad_driving")

        # Parameters
        self.speed = self.declare_parameter("speed", 1.0)
        self.turn_radius = self.declare_parameter("turn_radius", 2.5)
        self.frame_id = self.declare_parameter("frame_id", "base_footprint")

        # Subscribers and publishers
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_cb, qos_profile=10
        )
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, "cmd_vel", qos_profile=10
        )

        self.get_logger().info("Gamepad driving node started")

    def joy_cb(self, msg: Joy):
        """Process joystick input and publish cmd_vel."""
        # [-1, 1] inputs
        forward_input = (-msg.axes[RIGHT_TRIGGER] * 0.5 + 0.5)
        backward_input = (-msg.axes[LEFT_TRIGGER] * 0.5 + 0.5)
        linear_input = forward_input - backward_input
        turn_input = msg.axes[LEFT_X]
        
        # If no trigger input but stick is being used, provide small linear velocity
        if abs(linear_input) < 0.001 and abs(turn_input) > 0.001:
            linear_input = 0.001
        
        # Scale linear velocity by max speed
        linear_velocity = linear_input * self.speed.value
        angular_velocity = turn_input * linear_velocity / self.turn_radius.value
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id.value
        twist_stamped.twist.linear.x = linear_velocity
        twist_stamped.twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_stamped)

def main():
    try:
        rclpy.init()
        node = GamepadDriving()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
