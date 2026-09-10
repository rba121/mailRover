import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# Motor 1 Left
PWM1_PIN = 18   # PIN12
INA1_PIN = 17   # PIN11
INB1_PIN = 27   # PIN13

# Motor 2 Right
PWM2_PIN = 12   # PIN32
INA2_PIN = 22   # PIN15
INB2_PIN = 23   # PIN16

MAX_SPEED = 0.55
LEFT_BOOST = 1.10
RIGHT_BOOST = 1.00

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Left motor
        self.pwm1 = PWMOutputDevice(PWM1_PIN, frequency=1000, initial_value=0)
        self.ina1 = DigitalOutputDevice(INA1_PIN)
        self.inb1 = DigitalOutputDevice(INB1_PIN)

        # Right motor
        self.pwm2 = PWMOutputDevice(PWM2_PIN, frequency=1000, initial_value=0)
        self.ina2 = DigitalOutputDevice(INA2_PIN)
        self.inb2 = DigitalOutputDevice(INB2_PIN)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.safety_stop)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info('Motor Controller Node started')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive mix
	left_speed = (left_speed / peak) * MAX_SPEED * LEFT_BOOST
	right_speed = (right_speed / peak) * MAX_SPEED * RIGHT_BOOST

        # Normalize so neither exceeds MAX_SPEED
        peak = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed = (left_speed / peak) * MAX_SPEED
        right_speed = (right_speed / peak) * MAX_SPEED

        self.set_left_motor(left_speed)
        self.set_right_motor(right_speed)

        self.get_logger().info(
            f'cmd_vel -> linear={linear:.2f}, angular={angular:.2f}, '
            f'left={left_speed:.2f}, right={right_speed:.2f}'
        )

    def set_left_motor(self, speed):
        speed = max(-1.0, min(1.0, speed))

        if abs(speed) < 0.05:
            speed = 0.0

        if speed > 0:
            # forward
            self.ina1.on()
            self.inb1.off()
            self.pwm1.value = speed
        elif speed < 0:
            # backward
            self.ina1.off()
            self.inb1.on()
            self.pwm1.value = abs(speed)
        else:
            self.pwm1.value = 0.0
            self.ina1.on()
            self.inb1.on()

    def set_right_motor(self, speed):
        speed = max(-1.0, min(1.0, speed))

        if abs(speed) < 0.05:
            speed = 0.0

        if speed > 0:
            # forward for right motor is reversed polarity
            self.ina2.off()
            self.inb2.on()
            self.pwm2.value = speed
        elif speed < 0:
            # backward
            self.ina2.on()
            self.inb2.off()
            self.pwm2.value = abs(speed)
        else:
            self.pwm2.value = 0.0
            self.ina2.on()
            self.inb2.on()

    def safety_stop(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.set_left_motor(0.0)
            self.set_right_motor(0.0)

    def destroy_node(self):
        self.set_left_motor(0.0)
        self.set_right_motor(0.0)
        self.pwm1.close()
        self.pwm2.close()
        self.ina1.close()
        self.inb1.close()
        self.ina2.close()
        self.inb2.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()