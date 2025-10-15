import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from odrive_can.msg import ControlMessage


# AXES
LEFT_X = 0
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5
SET_TEMPOMAT_VEL = 6
SET_PIPES_VEL = 7

# BUTTONS
RIGHT_SHOULDER = 5
LEFT_SHOULDER = 4
CHANGE_MODE = 7

# TILT BUTTONS
TILT_DOWN = 0
TILT_UP  =  3

class GamepadDriving(Node):
    def __init__(self):
        super().__init__("gamepad_driving")

        # Parameters
        self.toggle_modes = ["MANUAL", "TEMPOMAT"]
        self.start_mode = self.declare_parameter("driving_mode", "MANUAL")
        self.robot_max_vel = self.declare_parameter("robot_max_vel", 0.25)
        self.pipe_max_vel = self.declare_parameter("pipe_max_vel", 10.0)
        self.pipe_max_tilt_angle = self.declare_parameter("pipe_max_tilt_angle", 2.0)
        self.turn_radius = self.declare_parameter("turn_radius", 1.5)
        self.frame_id = self.declare_parameter("frame_id", "base_footprint")
        self.tempomat_vel_step = self.declare_parameter("tempomat_vel_step", 0.05)
        self.pipe_vel_step = self.declare_parameter("pipe_vel_step", 5.00)
        self.pipe_tilt_step = self.declare_parameter("pipe_tilt_step", 0.05)

        self.stopped : bool = False
        self.drilling_state = False
        self.mode : str = self.start_mode.value
        self.mode_idx : int = 0
        self.tempomat_vel : float = 0.0
        self.pipe_vel : float = 0.0
        self.pipe_tilt : float = 0.0
        self.prev_mode: str = ""
        
        # Subscribers and publishers
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_cb, qos_profile=10
        )
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, "cmd_vel", qos_profile=10
        )
        self.pipe_vel_pub = self.create_publisher(
            ControlMessage, "/odrive_pipes/control_message", qos_profile=10
        )
        self.pipe_tilt_pub = self.create_publisher(
            ControlMessage, "/odrive_tilt/control_message", qos_profile=10
        )

        self.get_logger().info("Gamepad driving node started")

    def joy_cb(self, msg: Joy):
        """Process joystick input and publish cmd_vel."""
        # [-1, 1] inputs
        if not hasattr(self, 'prev_left_shoulder'):
            self.prev_left_shoulder = 0
        if not hasattr(self, 'prev_change_mode'):
            self.prev_change_mode = 0

        if msg.buttons[LEFT_SHOULDER] == 1.0 and self.prev_left_shoulder == 0:
            if not self.stopped:
                self.prev_mode = self.mode
                self.mode = "STOP"
                self.stopped = True
                self.tempomat_vel = 0.0
            else:
                self.mode = self.prev_mode
                self.stopped = False

        if msg.buttons[CHANGE_MODE] == 1.0 and self.prev_change_mode == 0:
            self.mode_idx = (self.mode_idx + 1) % len(self.toggle_modes)
            self.mode = self.toggle_modes[self.mode_idx]

        self.prev_left_shoulder = msg.buttons[LEFT_SHOULDER]
        self.prev_change_mode = msg.buttons[CHANGE_MODE]


        if self.mode == "STOP":
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = self.frame_id.value
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.pipe_vel = 0.0
            self.cmd_vel_pub.publish(twist_stamped)

        if self.mode == "TEMPOMAT":
            turn_input = msg.axes[LEFT_X]

            # Adjust tempomat velocity
            if msg.axes[SET_TEMPOMAT_VEL] == -1.0:
                self.tempomat_vel += self.tempomat_vel_step.value
            elif msg.axes[SET_TEMPOMAT_VEL] == 1.0:
                self.tempomat_vel -= self.tempomat_vel_step.value

            # Clamp velocity
            self.tempomat_vel = min(max(self.tempomat_vel, 0.0), self.robot_max_vel.value)

            
            # Scale linear velocity by max robot_max_vel
            linear_velocity = self.tempomat_vel
            angular_velocity = turn_input * linear_velocity / self.turn_radius.value
            
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = self.frame_id.value
            twist_stamped.twist.linear.x = linear_velocity
            twist_stamped.twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_stamped)



        if self.mode == "MANUAL":
            forward_input = (-msg.axes[RIGHT_TRIGGER] * 0.5 + 0.5)
            backward_input = (-msg.axes[LEFT_TRIGGER] * 0.5 + 0.5)
            linear_input = forward_input - backward_input
            turn_input = msg.axes[LEFT_X]
            
            # If no trigger input but stick is being used, provide small linear velocity
            if abs(linear_input) < 0.001 and abs(turn_input) > 0.001:
                linear_input = 0.001
            
            # Scale linear velocity by max robot_max_vel
            linear_velocity = linear_input * self.robot_max_vel.value
            angular_velocity = turn_input * linear_velocity / self.turn_radius.value
            
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = self.frame_id.value
            twist_stamped.twist.linear.x = linear_velocity
            twist_stamped.twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_stamped)
        
        if msg.buttons[RIGHT_SHOULDER] == 1.0 and not self.prev_shoulder and not self.stopped:
            self.drilling_state = not self.drilling_state
            control_msg = ControlMessage()
            if not self.drilling_state:
                control_msg.control_mode = 2
                control_msg.input_mode = 2
                control_msg.input_pos = 0.0
                control_msg.input_vel = 0.0
                control_msg.input_torque = 0.0
                self.pipe_vel_pub.publish(control_msg)
        self.prev_shoulder = msg.buttons[RIGHT_SHOULDER]

        if self.drilling_state:
            if msg.axes[SET_PIPES_VEL] == 1.0:
                self.pipe_vel += self.pipe_vel_step.value
            elif msg.axes[SET_PIPES_VEL] == -1.0:
                self.pipe_vel -= self.pipe_vel_step.value

            self.pipe_vel = min(max(self.pipe_vel, 0.0), self.pipe_max_vel.value)

            control_msg = ControlMessage()
            control_msg.control_mode = 2
            control_msg.input_mode = 2
            control_msg.input_pos = 0.0
            control_msg.input_vel = self.pipe_vel
            control_msg.input_torque = 0.0
            self.pipe_vel_pub.publish(control_msg)
        
        if self.mode == "MANUAL" or self.mode == "TEMPOMAT":
            if msg.buttons[TILT_DOWN] == 1.0:
                self.pipe_tilt += self.pipe_tilt_step.value
            elif msg.buttons[TILT_UP] == 1.0:
                self.pipe_tilt -= self.pipe_tilt_step.value

            self.pipe_tilt = min(max(self.pipe_tilt, 0.0), self.pipe_max_tilt_angle.value)

            control_msg = ControlMessage()
            control_msg.control_mode = 2
            control_msg.input_mode = 2
            control_msg.input_pos = self.pipe_tilt
            control_msg.input_vel = 0.0
            control_msg.input_torque = 0.0
            self.pipe_tilt_pub.publish(control_msg)
                
            
        



def main():
    try:
        rclpy.init()
        node = GamepadDriving()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
