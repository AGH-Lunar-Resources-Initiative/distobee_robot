import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from rclpy.qos import ReliabilityPolicy

import RPi.GPIO as GPIO

BRUSHES_DIRECTION_PIN = 15
BRUSHES_PWM_PIN = 14
FREQUENCY = 1000  # Frequency in Hz
VIBRATORS_PWM_PIN = 12

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
            'vibrator',
            self.vibrator_callback,
            ReliabilityPolicy.RELIABLE
        )
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BRUSHES_DIRECTION_PIN, GPIO.OUT)
        GPIO.setup(BRUSHES_PWM_PIN, GPIO.OUT)
        GPIO.setup(VIBRATORS_PWM_PIN, GPIO.OUT)

        # Initialize PWM
        self.brushes_pwm = GPIO.PWM(BRUSHES_PWM_PIN, FREQUENCY)
        self.brushes_pwm.start(0)

        self.vibrators_pwm = GPIO.PWM(VIBRATORS_PWM_PIN, FREQUENCY)
        self.vibrators_pwm.start(0)

    def brush_pwm_callback(self, msg: Int32):
        self.get_logger().info(f'Received brush PWM: {msg.data}')

        if msg.data < -100 or msg.data > 100:
            self.get_logger().warn(f'Brush PWM value {msg.data} out of range (-100 to 100)')
            return
        
        target_speed: int = msg.data

        if target_speed > 0:
            GPIO.output(BRUSHES_DIRECTION_PIN, GPIO.HIGH)
            self.brushes_pwm.ChangeDutyCycle(target_speed)
            self.get_logger().debug(f"Moving forward at speed {target_speed}%")
        elif target_speed < 0:
            GPIO.output(BRUSHES_DIRECTION_PIN, GPIO.LOW)
            self.brushes_pwm.ChangeDutyCycle(-target_speed)
            self.get_logger().debug(f"Moving backward at speed {-target_speed}%")
        else:
            self.brushes_pwm.ChangeDutyCycle(0)
            self.get_logger().debug("Motor stopped.")


    def vibrator_callback(self, msg: Bool):
        self.get_logger().info(f'Received vibrator command: {msg.data}')

        state: bool = msg.data

        if state is False:
            self.vibrators_pwm.ChangeDutyCycle(0)
            self.get_logger().debug("Vibrators turned off.")
        if state is True:
            self.vibrators_pwm.ChangeDutyCycle(100)
            self.get_logger().debug("Vibrators turned on at 100%.")    
        else:
            self.get_logger().debug("Invalid state. Please enter 1 or 0.")

    
def main():
    try:
        rclpy.init()
        node = SifterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
