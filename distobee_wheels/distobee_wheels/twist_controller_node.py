import numpy as np
import rclpy
import rclpy.node
from geometry_msgs.msg import TwistStamped
from distobee_interfaces.msg import WheelStates

# wheel positions relative to turning origin (m)
WHEEL_POSITIONS = [
    np.array([0.9, 0.35]),  # front left
    np.array([0.9, -0.35]),  # front right
    np.array([0.0, 0.35]),  # back left
    np.array([0.0, -0.35]),  # back right
]

TURN_VECTORS = [
    np.array([-wheel_pos[1], wheel_pos[0]])  # rot +90
    for wheel_pos in WHEEL_POSITIONS
]

# wheel radius in meters (for converting linear velocity to angular velocity)
WHEEL_RADIUS = 0.2  # 20cm radius - adjust this value based on actual wheel size

MAX_ACCEL = 0.5 # rad/s^2
MAX_TURN_ANGLE  = 1.2 # rad

def flip_angle(angle):
    angle += np.pi
    return np.arctan2(np.sin(angle), np.cos(angle))


class TwistController(rclpy.node.Node):
    def __init__(self):
        super().__init__("twist_controller")

        # Read parameters.
        self.declare_parameter("max_wheel_vel", 0.25)
        self.declare_parameter("wheel_accel", 0.5)
        self.declare_parameter("wheel_decel", 2.0)
        self.declare_parameter("wheel_turn_vel", 1.2)

        # Create subscribers.
        self.create_subscription(TwistStamped, "cmd_vel", self.on_cmd_vel, 10)

        # Create state publisher.
        self.wheel_state_pub = self.create_publisher(WheelStates, "wheel_states/target", 10)

        self.last_time = self.get_clock().now()
        self.smooth_velocity = 0.0

    def on_cmd_vel(self, msg: TwistStamped):
        # smooth velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # convert to seconds
        self.last_time = current_time
        if dt <= 0.0:
            dt = 1e-6  # Prevent division by zero
        max_delta = MAX_ACCEL * dt
        delta = msg.twist.linear.x - self.smooth_velocity
        if abs(delta) > max_delta:
            delta = np.sign(delta) * max_delta
        self.smooth_velocity += delta

        # linear motion vector
        linear_vector = np.array([self.smooth_velocity, 0.0])

        # angular motion vectors
        angular_vectors = [vec * msg.twist.angular.z for vec in TURN_VECTORS]

        # final wheel vectors
        wheel_vectors = [
            linear_vector + angular_vector for angular_vector in angular_vectors
        ]

        # wheel linear velocities in m/s
        wheel_linear_velocities = [np.linalg.norm(vec) for vec in wheel_vectors]

        # wheel angles
        wheel_angles = [np.arctan2(vec[1], vec[0]) for vec in wheel_vectors]

        # flip wheels that go backwards
        for i in range(len(wheel_linear_velocities)):
            if abs(wheel_angles[i]) > np.pi / 2:
                wheel_linear_velocities[i] = -wheel_linear_velocities[i]
                wheel_angles[i] = flip_angle(wheel_angles[i])


        # normalize wheel velocities
        max_wheel_vel = self.get_parameter("max_wheel_vel").value
        cur_max_vel = max(wheel_linear_velocities[2:])  # only back wheels drive
        if cur_max_vel > max_wheel_vel:
            wheel_linear_velocities = [
                vel * (max_wheel_vel / cur_max_vel) for vel in wheel_linear_velocities
            ]

        # convert linear velocities to angular velocities (rad/s)
        wheel_angular_velocities = [vel / WHEEL_RADIUS for vel in wheel_linear_velocities]

        # publish wheel states
        wheel_states = WheelStates()
        wheel_states.back_left_velocity = wheel_angular_velocities[2]   # rad/s
        wheel_states.back_right_velocity = wheel_angular_velocities[3]  # rad/s
        wheel_states.front_left_angle = wheel_angles[0]
        wheel_states.front_right_angle = wheel_angles[1]
        self.wheel_state_pub.publish(wheel_states)


def main():
    try:
        rclpy.init()
        node = TwistController()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass