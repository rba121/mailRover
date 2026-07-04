import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# Motor 1 Left
PWM1_PIN = 18
INA1_PIN = 17
INB1_PIN = 27

# Motor 2 Right
PWM2_PIN = 12
INA2_PIN = 22
INB2_PIN = 23
MIN_EFFECTIVE_PWM = 0.15
MAX_PWM = 0.20
DEADBAND = 0.02
DIST = 0.4572
VDES = 0.5

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.pwm1 = PWMOutputDevice(PWM1_PIN, frequency=1000, initial_value=0)
        self.ina1 = DigitalOutputDevice(INA1_PIN)
        self.inb1 = DigitalOutputDevice(INB1_PIN)

        self.pwm2 = PWMOutputDevice(PWM2_PIN, frequency=1000, initial_value=0)
        self.ina2 = DigitalOutputDevice(INA2_PIN)
        self.inb2 = DigitalOutputDevice(INB2_PIN)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.safety_stop)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info('Motor Controller Node started')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = (linear - (angular * (DIST / 2))) * (MAX_PWM / VDES)
        right_speed = (linear + (angular * (DIST / 2))) * (MAX_PWM / VDES)

       # Deadband
        if abs(left_speed) < DEADBAND:
            left_speed = 0.0
        if abs(right_speed) < DEADBAND:
            right_speed = 0.0

       # Minimum effective PWM for real motors
        if 0 < abs(left_speed) < MIN_EFFECTIVE_PWM:
            left_speed = MIN_EFFECTIVE_PWM if left_speed > 0 else -MIN_EFFECTIVE_PWM
        if 0 < abs(right_speed) < MIN_EFFECTIVE_PWM:
            right_speed = MIN_EFFECTIVE_PWM if right_speed > 0 else -MIN_EFFECTIVE_PWM


       # linear = msg.linear.x * 7.5
       # angular = msg.angular.z * ANGULAR_MULT

       # left_speed = linear - angular
       # right_speed = linear + angular

        # Apply trim always
       # left_speed *= LEFT_TRIM
       # right_speed *= RIGHT_TRIM

        # Apply max speed cap
       # left_speed *= MAX_SPEED
       # right_speed *= MAX_SPEED

        # Deadband
       # if abs(left_speed) < DEADBAND:
        #    left_speed = 0.0
       # if abs(right_speed) < DEADBAND:
        #    right_speed = 0.0

        self.set_left_motor(left_speed)
        self.set_right_motor(right_speed)

        self.get_logger().info(
            f'cmd_vel -> linear={linear:.2f}, angular={angular:.2f}, '
            f'left={left_speed:.2f}, right={right_speed:.2f}'
        )

    def set_left_motor(self, speed):
        speed = max(-0.25, min(0.25, speed))
        #self.get_logger().info(f'left={speed:.2f}')
        if speed > 0:
            self.ina1.on()
            self.inb1.off()
            self.pwm1.value = speed
        elif speed < 0:
            self.ina1.off()
            self.inb1.on()
            self.pwm1.value = abs(speed)
        else:
            self.ina1.off()
            self.inb1.off()
            self.pwm1.value = 0.0

    def set_right_motor(self, speed):
        speed = max(-0.25, min(0.25, speed))
        #self.get_logger().info(f'right={speed:.2f}')
        if speed > 0:
            self.ina2.off()
            self.inb2.on()
            self.pwm2.value = speed
        elif speed < 0:
            self.ina2.on()
            self.inb2.off()
            self.pwm2.value = abs(speed)
        else:
            self.ina2.off()
            self.inb2.off()
            self.pwm2.value = 0.0

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
