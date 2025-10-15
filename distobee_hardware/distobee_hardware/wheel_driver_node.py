import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from distobee_interfaces.msg import WheelStates, WheelTelemetry
from odrive_can.msg import ControlMessage, ControllerStatus

ACCEL = 1.0  # rad/s^2
VEL_SCALE = 120 / 6.28 # post-gearbox rad/s -> pre-gearbox rev/s
M_PI = 3.1415926


class WheelDriver(rclpy.node.Node):
    """
    Node that interfaces with 4 ODrive controllers to control robot wheels.
    Subscribes to wheel_states/target and publishes wheel_states/current.

    Units:
    - Drive wheel velocities: rad/s (angular velocity)
    - Steer wheel angles: radians (position)

    Note: ODrive initialization (setting to CLOSED_LOOP_CONTROL state) is handled
    by the launch file. This node only sets up control modes and handles wheel commands.
    """

    def __init__(self):
        super().__init__("wheel_driver")

        # Initialize wheel state tracking
        self.current_wheel_states = WheelStates()

        # Status tracking for each ODrive
        self.back_left_status_received = False
        self.back_right_status_received = False
        self.front_left_status_received = False
        self.front_right_status_received = False

        # QoS profiles
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribe to target wheel states
        self.create_subscription(
            WheelStates, "wheel_states/target", self.on_target_wheel_states, cmd_qos
        )

        # Publish current wheel states
        self.wheel_states_pub = self.create_publisher(
            WheelStates, "wheel_states/current", status_qos
        )

        # Publish current wheel telemetry, all on dedicated topic
        self.wheel_telemetry_fl_pub = self.create_publisher(
            WheelTelemetry, "wheel_states/telemetry/fl", status_qos
        )
        self.wheel_telemetry_fr_pub = self.create_publisher(
            WheelTelemetry, "wheel_states/telemetry/fr", status_qos
        )
        self.wheel_telemetry_bl_pub = self.create_publisher(
            WheelTelemetry, "wheel_states/telemetry/bl", status_qos
        )
        self.wheel_telemetry_br_pub = self.create_publisher(
            WheelTelemetry, "wheel_states/telemetry/br", status_qos
        )

        # Publishers for ODrive control messages
        self.back_left_control_pub = self.create_publisher(
            ControlMessage, "/odrive_back_left/control_message", cmd_qos
        )
        self.back_right_control_pub = self.create_publisher(
            ControlMessage, "/odrive_back_right/control_message", cmd_qos
        )
        self.front_left_control_pub = self.create_publisher(
            ControlMessage, "/odrive_front_left/control_message", cmd_qos
        )
        self.front_right_control_pub = self.create_publisher(
            ControlMessage, "/odrive_front_right/control_message", cmd_qos
        )

        # Subscribers for ODrive status messages
        self.create_subscription(
            ControllerStatus,
            "/odrive_back_left/controller_status",
            self.on_back_left_status,
            status_qos,
        )
        self.create_subscription(
            ControllerStatus,
            "/odrive_back_right/controller_status",
            self.on_back_right_status,
            status_qos,
        )
        self.create_subscription(
            ControllerStatus,
            "/odrive_front_left/controller_status",
            self.on_front_left_status,
            status_qos,
        )
        self.create_subscription(
            ControllerStatus,
            "/odrive_front_right/controller_status",
            self.on_front_right_status,
            status_qos,
        )

        # Publish current wheel states periodically
        self.create_timer(0.05, self.publish_current_states)  # 20 Hz

        self.last_time = self.get_clock().now()
        self.smooth_velocity_left = 0.0
        self.smooth_velocity_right = 0.0

    def on_target_wheel_states(self, msg: WheelStates):
        # Back left wheel (drive)
        back_left_msg = ControlMessage()
        back_left_msg.control_mode = 2  # VELOCITY_CONTROL
        back_left_msg.input_mode = 1  # PASSTHROUGH
        back_left_msg.input_vel = -msg.back_left_velocity * VEL_SCALE  # rad/s
        back_left_msg.input_pos = 0.0
        back_left_msg.input_torque = 0.0
        self.back_left_control_pub.publish(back_left_msg)

        # Back right wheel (drive)
        back_right_msg = ControlMessage()
        back_right_msg.control_mode = 2  # VELOCITY_CONTROL
        back_right_msg.input_mode = 1  # PASSTHROUGH
        back_right_msg.input_vel = msg.back_right_velocity * VEL_SCALE  # rad/s
        back_right_msg.input_pos = 0.0
        back_right_msg.input_torque = 0.0
        self.back_right_control_pub.publish(back_right_msg)

        # Front left wheel (steer)
        front_left_msg = ControlMessage()
        front_left_msg.control_mode = 3  # POSITION_CONTROL
        front_left_msg.input_mode = 1  # PASSTHROUGH
        front_left_msg.input_pos = msg.front_left_angle / (2 * M_PI)  # No offset necessary, everything is done in odrive firmware
        front_left_msg.input_vel = 0.0
        front_left_msg.input_torque = 0.0
        self.front_left_control_pub.publish(front_left_msg)

        # Front right wheel (steer)
        front_right_msg = ControlMessage()
        front_right_msg.control_mode = 3  # POSITION_CONTROL
        front_right_msg.input_mode = 1  # PASSTHROUGH
        front_right_msg.input_pos = msg.front_right_angle / (2 * M_PI)  # No offset necessary, everything is done in odrive firmware

        front_right_msg.input_vel = 0.0
        front_right_msg.input_torque = 0.0
        self.front_right_control_pub.publish(front_right_msg)

    def on_back_left_status(self, msg: ControllerStatus):
        """Update current back left wheel state"""
        self.current_wheel_states.back_left_velocity = -msg.vel_estimate / VEL_SCALE  # rad/s
        if not self.back_left_status_received:
            self.back_left_status_received = True
            self.get_logger().info("Receiving back left ODrive status")
        wheel_telemetry = WheelTelemetry()
        wheel_telemetry.state = msg.axis_state
        self.wheel_telemetry_bl_pub.publish(wheel_telemetry)

    def on_back_right_status(self, msg: ControllerStatus):
        """Update current back right wheel state"""
        self.current_wheel_states.back_right_velocity = msg.vel_estimate / VEL_SCALE  # rad/s
        if not self.back_right_status_received:
            self.back_right_status_received = True
            self.get_logger().info("Receiving back right ODrive status")
        wheel_telemetry = WheelTelemetry()
        wheel_telemetry.state = msg.axis_state
        self.wheel_telemetry_br_pub.publish(wheel_telemetry)

    def on_front_left_status(self, msg: ControllerStatus):
        """Update current front left wheel state"""
        self.current_wheel_states.front_left_angle = msg.pos_estimate
        if not self.front_left_status_received:
            self.front_left_status_received = True
            self.get_logger().info("Receiving front left ODrive status")
        wheel_telemetry = WheelTelemetry()
        wheel_telemetry.state = msg.axis_state
        self.wheel_telemetry_fl_pub.publish(wheel_telemetry)

    def on_front_right_status(self, msg: ControllerStatus):
        """Update current front right wheel state"""
        self.current_wheel_states.front_right_angle = msg.pos_estimate
        if not self.front_right_status_received:
            self.front_right_status_received = True
            self.get_logger().info("Receiving front right ODrive status")
        wheel_telemetry = WheelTelemetry()
        wheel_telemetry.state = msg.axis_state
        self.wheel_telemetry_fr_pub.publish(wheel_telemetry)

    def publish_current_states(self):
        """Publish current wheel states"""
        # Only publish if we have received at least some status messages
        if (
            self.back_left_status_received
            or self.back_right_status_received
            or self.front_left_status_received
            or self.front_right_status_received
        ):
            self.wheel_states_pub.publish(self.current_wheel_states)


def main():
    try:
        rclpy.init()
        node = WheelDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
