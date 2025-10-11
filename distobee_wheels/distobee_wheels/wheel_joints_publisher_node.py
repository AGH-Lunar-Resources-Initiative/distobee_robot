import rclpy
import rclpy.node
from sensor_msgs.msg import JointState
from distobee_interfaces.msg import WheelStates


class WheelJointsPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("wheel_joints_publisher")

        # Initialize wheel position variables for simulation
        self.wheel_bl_position = 0.0  # back left wheel position (radians)
        self.wheel_br_position = 0.0  # back right wheel position (radians)
        self.wheel_fl_position = 0.0  # front left wheel position (radians)
        self.wheel_fr_position = 0.0  # front right wheel position (radians)
        
        # Store last timestamp for velocity integration
        self.last_time = None

        # Create subscriber for wheel states
        self.create_subscription(WheelStates, "wheel_states/target", self.on_wheel_states, 10)
        # ^ TODO: change to current

        # Create publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, "wheel_states/current/joint_states", 10)

    def on_wheel_states(self, msg: WheelStates):
        """Convert WheelStates to JointState for RViz visualization."""
        current_time = self.get_clock().now()
        
        # Calculate time delta for velocity integration
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9  # convert to seconds
            
            # Integrate velocities to get wheel positions
            self.wheel_bl_position += msg.back_left_velocity * dt
            self.wheel_br_position += msg.back_right_velocity * dt
            # Front wheels don't drive, so their positions remain 0 or could follow steering
            self.wheel_fl_position += 0.0 * dt
            self.wheel_fr_position += 0.0 * dt
        
        self.last_time = current_time
        
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()

        # Define joint names matching the robot description
        joint_state.name = [
            "turn_fl_joint",  # front left steering joint
            "turn_fr_joint",  # front right steering joint
            "wheel_bl_joint",  # back left wheel joint
            "wheel_br_joint",  # back right wheel joint
            "wheel_fl_joint",  # front left wheel joint
            "wheel_fr_joint",  # front right wheel joint
        ]

        # Set joint positions/angles
        joint_state.position = [
            msg.front_left_angle,      # front left steering angle
            msg.front_right_angle,     # front right steering angle
            self.wheel_bl_position,    # back left wheel position (simulated)
            self.wheel_br_position,    # back right wheel position (simulated)
            self.wheel_fl_position,    # front left wheel position (simulated)
            self.wheel_fr_position,    # front right wheel position (simulated)
        ]

        # Set joint velocities
        joint_state.velocity = [
            0.0,                       # front left steering velocity (not needed)
            0.0,                       # front right steering velocity (not needed)
            msg.back_left_velocity,    # back left wheel velocity
            msg.back_right_velocity,   # back right wheel velocity
            0.0,                       # front left wheel velocity (front wheels don't drive)
            0.0,                       # front right wheel velocity (front wheels don't drive)
        ]

        # Publish joint state
        self.joint_state_pub.publish(joint_state)


def main():
    try:
        rclpy.init()
        node = WheelJointsPublisher()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
