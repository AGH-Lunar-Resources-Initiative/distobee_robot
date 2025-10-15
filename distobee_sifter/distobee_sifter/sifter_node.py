import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from rclpy.qos import ReliabilityPolicy
from distobee_sifter.sifter_motors_command import SifterMotorsCommand
from distobee_sifter.client import Client

class SifterNode(Node):
    def __init__(self):
        super().__init__('sifter_node')
        self.get_logger().info('Sifter node has been started.')

        self.brush_pwm_sub = self.create_subscription(
            Int32,
            'brush_pwm',
            self.brush_pwm_callback,
            ReliabilityPolicy.RELIABLE
        )

        self.vibrator_sub = self.create_subscription(
            Bool,
            'vibrators',
            self.vibrator_callback,
            ReliabilityPolicy.RELIABLE
        )

        self.declare_parameter("tcp_host", "192.168.1.43")
        self.declare_parameter("tcp_port", 6000)

        self.host = self.get_parameter("tcp_host").value
        self.port = self.get_parameter("tcp_port").get_parameter_value().integer_value

        self.client = Client(self.host, self.port)

    def brush_pwm_callback(self, msg: Int32):
        self.get_logger().info(f'Received brush PWM: {msg.data}')

        if msg.data < -100 or msg.data > 100:
            self.get_logger().warn(f'Brush PWM value {msg.data} out of range (-100 to 100)')
            return
        
        command = SifterMotorsCommand(brush_pwm=msg.data, vibration=None)
        self.client.send_command(command)


    def vibrator_callback(self, msg: Bool):
        self.get_logger().info(f'Received vibrator command: {msg.data}')

        state: bool = msg.data
        command = SifterMotorsCommand(brush_pwm=None, vibration=state)
        self.client.send_command(command)

    
def main():
    try:
        rclpy.init()
        node = SifterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
