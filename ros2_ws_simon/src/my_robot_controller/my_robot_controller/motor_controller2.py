import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# Motor 1 Left
PWM1_PIN = 13
INA1_PIN = 24
INB1_PIN = 23

# Motor 2 Right
PWM2_PIN = 12
INA2_PIN = 6
INB2_PIN = 5

# E-S Motor 42PG-775 geared motors used on this rover:
# 24 V, 1:49 gearbox, 165 rpm no-load, 130 rpm rated-load, AB Hall encoder.
# The encoder is not read by this node yet; RF2O/Nav2 are still providing odom.
DEFAULT_TRACK_WIDTH_M = 0.4572
DEFAULT_MAX_LINEAR_MPS = 0.50
DEFAULT_MIN_EFFECTIVE_PWM = 0.15
DEFAULT_MAX_PWM = 0.60
DEFAULT_DEADBAND = 0.02
DEFAULT_PWM_SLEW_PER_SEC = 0.35
DEFAULT_COMMAND_TIMEOUT_SEC = 0.5


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.declare_parameter('track_width_m', DEFAULT_TRACK_WIDTH_M)
        self.declare_parameter('max_linear_mps', DEFAULT_MAX_LINEAR_MPS)
        self.declare_parameter('min_effective_pwm', DEFAULT_MIN_EFFECTIVE_PWM)
        self.declare_parameter('max_pwm', DEFAULT_MAX_PWM)
        self.declare_parameter('deadband', DEFAULT_DEADBAND)
        self.declare_parameter('pwm_slew_per_sec', DEFAULT_PWM_SLEW_PER_SEC)
        self.declare_parameter('command_timeout_sec', DEFAULT_COMMAND_TIMEOUT_SEC)
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 1.0)
        self.declare_parameter('left_motor_direction', 1.0)
        self.declare_parameter('right_motor_direction', 1.0)
        self.declare_parameter('pwm_frequency', 10000)

        self.track_width_m = float(self.get_parameter('track_width_m').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.min_effective_pwm = float(self.get_parameter('min_effective_pwm').value)
        self.max_pwm = float(self.get_parameter('max_pwm').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.pwm_slew_per_sec = float(self.get_parameter('pwm_slew_per_sec').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.left_trim = float(self.get_parameter('left_trim').value)
        self.right_trim = float(self.get_parameter('right_trim').value)
        self.left_motor_direction = float(
            self.get_parameter('left_motor_direction').value
        )
        self.right_motor_direction = float(
            self.get_parameter('right_motor_direction').value
        )
        pwm_frequency = int(self.get_parameter('pwm_frequency').value)

        self.target_left_pwm = 0.0
        self.target_right_pwm = 0.0
        self.applied_left_pwm = 0.0
        self.applied_right_pwm = 0.0

        self.pwm1 = PWMOutputDevice(PWM1_PIN, frequency=pwm_frequency, initial_value=0)
        self.ina1 = DigitalOutputDevice(INA1_PIN)
        self.inb1 = DigitalOutputDevice(INB1_PIN)

        self.pwm2 = PWMOutputDevice(PWM2_PIN, frequency=pwm_frequency, initial_value=0)
        self.ina2 = DigitalOutputDevice(INA2_PIN)
        self.inb2 = DigitalOutputDevice(INB2_PIN)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.apply_motor_outputs)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info(
            'Motor Controller Node started for 24 V geared motors '
            f'(max_pwm={self.max_pwm:.2f}, min_effective_pwm={self.min_effective_pwm:.2f}, '
            f'max_linear_mps={self.max_linear_mps:.2f})'
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = self.clamp(msg.linear.x, -self.max_linear_mps, self.max_linear_mps)
        angular = msg.angular.z

        left_mps = linear - (angular * (self.track_width_m / 2.0))
        right_mps = linear + (angular * (self.track_width_m / 2.0))

        left_pwm = (left_mps / self.max_linear_mps) * self.max_pwm
        right_pwm = (right_mps / self.max_linear_mps) * self.max_pwm

        left_pwm *= self.left_trim * self.left_motor_direction
        right_pwm *= self.right_trim * self.right_motor_direction

        self.target_left_pwm = self.prepare_pwm(left_pwm)
        self.target_right_pwm = self.prepare_pwm(right_pwm)

        self.get_logger().debug(
            f'cmd_vel -> linear={linear:.2f}, angular={angular:.2f}, '
            f'target_left={self.target_left_pwm:.2f}, target_right={self.target_right_pwm:.2f}'
        )

    def prepare_pwm(self, pwm):
        pwm = self.clamp(pwm, -self.max_pwm, self.max_pwm)
        if abs(pwm) < self.deadband:
            return 0.0
        if abs(pwm) < self.min_effective_pwm:
            return self.min_effective_pwm if pwm > 0 else -self.min_effective_pwm
        return pwm

    def apply_motor_outputs(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.command_timeout_sec:
            self.target_left_pwm = 0.0
            self.target_right_pwm = 0.0

        max_step = self.pwm_slew_per_sec * self.timer_period
        self.applied_left_pwm = self.slew_toward(
            self.applied_left_pwm,
            self.target_left_pwm,
            max_step,
        )
        self.applied_right_pwm = self.slew_toward(
            self.applied_right_pwm,
            self.target_right_pwm,
            max_step,
        )

        self.set_left_motor(self.applied_left_pwm)
        self.set_right_motor(self.applied_right_pwm)

    def slew_toward(self, current, target, max_step):
        delta = self.clamp(target - current, -max_step, max_step)
        next_value = current + delta
        if abs(target) < self.deadband and abs(next_value) < self.deadband:
            return 0.0
        return next_value

    def clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def set_left_motor(self, speed):
        speed = self.clamp(speed, -self.max_pwm, self.max_pwm)
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
        speed = self.clamp(speed, -self.max_pwm, self.max_pwm)

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

    def destroy_node(self):
        self.target_left_pwm = 0.0
        self.target_right_pwm = 0.0
        self.applied_left_pwm = 0.0
        self.applied_right_pwm = 0.0
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
