import rclpy
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from rclpy.node import Node
from sensor_msgs.msg import JointState

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
DEFAULT_TRACK_WIDTH_M = 0.4572
DEFAULT_MAX_LINEAR_MPS = 0.50
DEFAULT_MIN_EFFECTIVE_PWM = 0.15
DEFAULT_MAX_PWM = 0.60
DEFAULT_DEADBAND = 0.02
DEFAULT_PWM_SLEW_PER_SEC = 0.35
DEFAULT_COMMAND_TIMEOUT_SEC = 0.5
DEFAULT_ENCODER_TIMEOUT_SEC = 0.5
DEFAULT_WHEEL_RADIUS_M = 0.0635
DEFAULT_KP = 0.6
DEFAULT_KI = 0.1
DEFAULT_KD = 0.0
DEFAULT_I_CLAMP = 0.3


class PIDController:
    def __init__(self, kp, ki, kd, i_clamp):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_clamp = i_clamp
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        self.integral = max(-self.i_clamp, min(self.i_clamp, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


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
        self.declare_parameter('encoder_timeout_sec', DEFAULT_ENCODER_TIMEOUT_SEC)
        self.declare_parameter('wheel_radius_m', DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter('use_closed_loop', True)
        self.declare_parameter('kp', DEFAULT_KP)
        self.declare_parameter('ki', DEFAULT_KI)
        self.declare_parameter('kd', DEFAULT_KD)
        self.declare_parameter('i_clamp', DEFAULT_I_CLAMP)
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
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.encoder_timeout_sec = float(
            self.get_parameter('encoder_timeout_sec').value
        )
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.use_closed_loop = bool(self.get_parameter('use_closed_loop').value)
        self.left_trim = float(self.get_parameter('left_trim').value)
        self.right_trim = float(self.get_parameter('right_trim').value)
        self.left_motor_direction = float(
            self.get_parameter('left_motor_direction').value
        )
        self.right_motor_direction = float(
            self.get_parameter('right_motor_direction').value
        )
        pwm_frequency = int(self.get_parameter('pwm_frequency').value)
        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        i_clamp = float(self.get_parameter('i_clamp').value)

        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.target_left_pwm = 0.0
        self.target_right_pwm = 0.0
        self.applied_left_pwm = 0.0
        self.applied_right_pwm = 0.0
        self.actual_left_mps = 0.0
        self.actual_right_mps = 0.0
        self.encoder_feedback_seen = False
        self.encoder_timeout_logged = False

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

        self.encoder_subscription = self.create_subscription(
            JointState,
            'joint_states/wheels',
            self.joint_state_callback,
            10,
        )

        self.left_pid = PIDController(kp, ki, kd, i_clamp)
        self.right_pid = PIDController(kp, ki, kd, i_clamp)

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.apply_motor_outputs)
        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_encoder_time = now
        self.last_loop_time = now

        self.get_logger().info(
            'Motor Controller Node started for 24 V geared motors '
            f'(max_pwm={self.max_pwm:.2f}, '
            f'min_effective_pwm={self.min_effective_pwm:.2f}, '
            f'max_linear_mps={self.max_linear_mps:.2f}, '
            f'closed_loop={self.use_closed_loop})'
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = self.clamp(msg.linear.x, -self.max_linear_mps, self.max_linear_mps)
        angular = msg.angular.z

        left_mps = linear - (angular * (self.track_width_m / 2.0))
        right_mps = linear + (angular * (self.track_width_m / 2.0))

        self.target_left_mps = left_mps
        self.target_right_mps = right_mps

        left_pwm = (left_mps / self.max_linear_mps) * self.max_pwm
        right_pwm = (right_mps / self.max_linear_mps) * self.max_pwm

        left_pwm *= self.left_trim * self.left_motor_direction
        right_pwm *= self.right_trim * self.right_motor_direction

        self.target_left_pwm = self.prepare_pwm(left_pwm)
        self.target_right_pwm = self.prepare_pwm(right_pwm)

        self.get_logger().debug(
            f'cmd_vel -> linear={linear:.2f}, angular={angular:.2f}, '
            f'target_left={self.target_left_pwm:.2f}, '
            f'target_right={self.target_right_pwm:.2f}'
        )

    def joint_state_callback(self, msg: JointState):
        try:
            left_idx = msg.name.index('left_wheel')
            right_idx = msg.name.index('right_wheel')
        except ValueError:
            return

        if left_idx >= len(msg.velocity) or right_idx >= len(msg.velocity):
            return

        self.actual_left_mps = msg.velocity[left_idx] * self.wheel_radius_m
        self.actual_right_mps = msg.velocity[right_idx] * self.wheel_radius_m
        self.last_encoder_time = self.get_clock().now()
        self.encoder_feedback_seen = True
        self.encoder_timeout_logged = False

    def prepare_pwm(self, pwm):
        pwm = self.clamp(pwm, -self.max_pwm, self.max_pwm)
        if abs(pwm) < self.deadband:
            return 0.0
        if abs(pwm) < self.min_effective_pwm:
            return self.min_effective_pwm if pwm > 0 else -self.min_effective_pwm
        return pwm

    def apply_motor_outputs(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        cmd_elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if cmd_elapsed > self.command_timeout_sec:
            self.target_left_mps = 0.0
            self.target_right_mps = 0.0
            self.target_left_pwm = 0.0
            self.target_right_pwm = 0.0
            self.left_pid.reset()
            self.right_pid.reset()

        if self.use_closed_loop:
            self.apply_closed_loop_outputs(now, dt)
        else:
            self.apply_open_loop_outputs()

    def apply_open_loop_outputs(self):
        self.applied_left_pwm = self.slew_pwm(
            self.applied_left_pwm,
            self.target_left_pwm,
        )
        self.applied_right_pwm = self.slew_pwm(
            self.applied_right_pwm,
            self.target_right_pwm,
        )

        self.set_left_motor(self.applied_left_pwm)
        self.set_right_motor(self.applied_right_pwm)

    def apply_closed_loop_outputs(self, now, dt):
        encoder_elapsed = (now - self.last_encoder_time).nanoseconds / 1e9
        if (
            not self.encoder_feedback_seen
            or encoder_elapsed > self.encoder_timeout_sec
        ):
            self.stop_outputs()
            target_nonzero = (
                abs(self.target_left_mps) > self.deadband
                or abs(self.target_right_mps) > self.deadband
            )
            if target_nonzero and not self.encoder_timeout_logged:
                self.get_logger().warn(
                    'No fresh encoder feedback; stopping closed-loop motor output'
                )
                self.encoder_timeout_logged = True
            return

        if (
            abs(self.target_left_mps) < self.deadband
            and abs(self.target_right_mps) < self.deadband
        ):
            self.stop_outputs()
            return

        target_left_pwm = self.closed_loop_pwm(
            self.target_left_mps,
            self.actual_left_mps,
            self.left_pid,
            dt,
        )
        target_right_pwm = self.closed_loop_pwm(
            self.target_right_mps,
            self.actual_right_mps,
            self.right_pid,
            dt,
        )

        self.applied_left_pwm = self.slew_pwm(self.applied_left_pwm, target_left_pwm)
        self.applied_right_pwm = self.slew_pwm(
            self.applied_right_pwm,
            target_right_pwm,
        )

        self.set_left_motor(
            self.applied_left_pwm * self.left_trim * self.left_motor_direction
        )
        self.set_right_motor(
            self.applied_right_pwm * self.right_trim * self.right_motor_direction
        )

        self.get_logger().debug(
            f'L target={self.target_left_mps:.2f} actual={self.actual_left_mps:.2f} '
            f'pwm={self.applied_left_pwm:.2f} | '
            f'R target={self.target_right_mps:.2f} actual={self.actual_right_mps:.2f} '
            f'pwm={self.applied_right_pwm:.2f}'
        )

    def closed_loop_pwm(self, target_mps, actual_mps, pid, dt):
        feedforward_pwm = (target_mps / self.max_linear_mps) * self.max_pwm
        correction_pwm = pid.compute(target_mps - actual_mps, dt)
        return self.prepare_pwm(feedforward_pwm + correction_pwm)

    def slew_pwm(self, current, target):
        max_step = self.pwm_slew_per_sec * self.timer_period
        return self.slew_toward(current, target, max_step)

    def stop_outputs(self):
        self.target_left_pwm = 0.0
        self.target_right_pwm = 0.0
        self.applied_left_pwm = 0.0
        self.applied_right_pwm = 0.0
        self.left_pid.reset()
        self.right_pid.reset()
        self.set_left_motor(0.0)
        self.set_right_motor(0.0)

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
