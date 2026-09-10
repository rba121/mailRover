import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import lgpio

# ---- Motor 1 (Left) - DFR0601 channel 1 ----
PWM1_PIN = 18
INA1_PIN = 17
INB1_PIN = 27

# ---- Motor 2 (Right) - DFR0601 channel 2 ----
PWM2_PIN = 12
INA2_PIN = 22
INB2_PIN = 23

MAX_SPEED = 0.43  # m/s - tune this to your robot's actual max speed


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller')

        # Motor 1 (Left) setup
        self.pwm1 = PWMOutputDevice(PWM1_PIN, frequency=1000, initial_value=0)
        self.ina1 = DigitalOutputDevice(INA1_PIN)
        self.inb1 = DigitalOutputDevice(INB1_PIN)

        # Motor 2 (Right) setup
        self.pwm2 = PWMOutputDevice(PWM2_PIN, frequency=1000, initial_value=0)
        self.ina2 = DigitalOutputDevice(INA2_PIN)
        self.inb2 = DigitalOutputDevice(INB2_PIN)

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Safety timer - stop motors if no cmd_vel received for 0.5s
        self.timer = self.create_timer(0.5, self.safety_stop)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info('Motor Controller Node started')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive mixing
        left_speed = linear - angular
        right_speed = linear + angular

        # Normalize to -1.0 to 1.0
        max_val = max(abs(left_speed), abs(right_speed), MAX_SPEED)
        left_speed /= max_val
        right_speed /= max_val

        self.set_motor(
            self.pwm1, self.ina1, self.inb1, left_speed)
        self.set_motor(
            self.pwm2, self.ina2, self.inb2, -right_speed)

        self.get_logger().debug(
            f'L={left_speed:.2f} R={right_speed:.2f}')

    def set_motor(self, pwm, ina, inb, speed):
        speed = max(-1.0, min(1.0, speed))
        if speed > 0:
            ina.on()
            inb.off()
            pwm.value = speed
        elif speed < 0:
            ina.off()
            inb.on()
            pwm.value = abs(speed)
        else:
            ina.off()
            inb.off()
            pwm.value = 0

    def safety_stop(self):
        # Stop motors if no cmd_vel received recently
        elapsed = (self.get_clock().now() -
                   self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.set_motor(self.pwm1, self.ina1, self.inb1, 0.0)
            self.set_motor(self.pwm2, self.ina2, self.inb2, 0.0)

    def destroy_node(self):
        # Clean up GPIO on shutdown
        self.set_motor(self.pwm1, self.ina1, self.inb1, 0.0)
        self.set_motor(self.pwm2, self.ina2, self.inb2, 0.0)
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
