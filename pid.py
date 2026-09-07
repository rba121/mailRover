import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# GPIO pins
PWM1_PIN = 12
INA1_PIN = 5
INB1_PIN = 6

PWM2_PIN = 13
INA2_PIN = 23
INB2_PIN = 24

DEFAULT_TRACK_WIDTH_M       = 0.3937 #0.4572
DEFAULT_WHEEL_RADIUS_M      = 0.0635
DEFAULT_MAX_LINEAR_MPS      = 0.30
DEFAULT_MIN_EFFECTIVE_PWM   = 0.15
DEFAULT_MAX_PWM             = 0.45
DEFAULT_DEADBAND            = 0.02
DEFAULT_COMMAND_TIMEOUT_SEC = 0.5
DEFAULT_FEEDBACK_TIMEOUT_SEC = 0.5
DEFAULT_PWM_SLEW_PER_SEC    = 0.35

DEFAULT_KP      = 0.4
DEFAULT_KI      = 0.1
DEFAULT_KD      = 0.0
DEFAULT_I_CLAMP = 0.3


class PIDController:
    def __init__(self, kp, ki, kd, i_clamp):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.i_clamp = i_clamp
        self.integral   = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self.integral  += error * dt
        self.integral   = max(-self.i_clamp, min(self.i_clamp, self.integral))
        derivative      = (error - self.prev_error) / dt
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0


class MotorControllerPIDNode(Node):
    def __init__(self):
        super().__init__('motor_controller_pid')

        self.declare_parameter('track_width_m',         DEFAULT_TRACK_WIDTH_M)
        self.declare_parameter('wheel_radius_m',        DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter('max_linear_mps',        DEFAULT_MAX_LINEAR_MPS)
        self.declare_parameter('min_effective_pwm',     DEFAULT_MIN_EFFECTIVE_PWM)
        self.declare_parameter('max_pwm',               DEFAULT_MAX_PWM)
        self.declare_parameter('deadband',              DEFAULT_DEADBAND)
        self.declare_parameter('command_timeout_sec',   DEFAULT_COMMAND_TIMEOUT_SEC)
        self.declare_parameter('feedback_timeout_sec',  DEFAULT_FEEDBACK_TIMEOUT_SEC)
        self.declare_parameter('pwm_slew_per_sec',      DEFAULT_PWM_SLEW_PER_SEC)
        self.declare_parameter('cmd_vel_topic',         'cmd_vel')
        self.declare_parameter('left_motor_direction',  -1.0)
        self.declare_parameter('right_motor_direction', 1.0)
        self.declare_parameter('pwm_frequency',         10000)
        self.declare_parameter('kp',      DEFAULT_KP)
        self.declare_parameter('ki',      DEFAULT_KI)
        self.declare_parameter('kd',      DEFAULT_KD)
        self.declare_parameter('i_clamp', DEFAULT_I_CLAMP)

        self.track_width_m         = float(self.get_parameter('track_width_m').value)
        self.wheel_radius_m        = float(self.get_parameter('wheel_radius_m').value)
        self.max_linear_mps        = float(self.get_parameter('max_linear_mps').value)
        self.min_effective_pwm     = float(self.get_parameter('min_effective_pwm').value)
        self.max_pwm               = float(self.get_parameter('max_pwm').value)
        self.deadband              = float(self.get_parameter('deadband').value)
        self.command_timeout_sec   = float(self.get_parameter('command_timeout_sec').value)
        self.feedback_timeout_sec  = float(self.get_parameter('feedback_timeout_sec').value)
        self.pwm_slew_per_sec      = float(self.get_parameter('pwm_slew_per_sec').value)
        cmd_vel_topic              = self.get_parameter('cmd_vel_topic').value
        self.left_motor_direction  = float(self.get_parameter('left_motor_direction').value)
        self.right_motor_direction = float(self.get_parameter('right_motor_direction').value)
        self.left_feedback_direction = self.left_motor_direction
        self.right_feedback_direction = self.right_motor_direction
        pwm_frequency              = int(self.get_parameter('pwm_frequency').value)

        kp      = float(self.get_parameter('kp').value)
        ki      = float(self.get_parameter('ki').value)
        kd      = float(self.get_parameter('kd').value)
        i_clamp = float(self.get_parameter('i_clamp').value)

        self.left_pid  = PIDController(kp, ki, kd, i_clamp)
        self.right_pid = PIDController(kp, ki, kd, i_clamp)

        self.target_left_mps  = 0.0
        self.target_right_mps = 0.0
        self.actual_left_mps  = 0.0
        self.actual_right_mps = 0.0
        self.applied_left_pwm  = 0.0
        self.applied_right_pwm = 0.0
        self.feedback_received = False

        self.pwm1 = PWMOutputDevice(PWM1_PIN, frequency=pwm_frequency, initial_value=0)
        self.ina1 = DigitalOutputDevice(INA1_PIN)
        self.inb1 = DigitalOutputDevice(INB1_PIN)
        self.pwm2 = PWMOutputDevice(PWM2_PIN, frequency=pwm_frequency, initial_value=0)
        self.ina2 = DigitalOutputDevice(INA2_PIN)
        self.inb2 = DigitalOutputDevice(INB2_PIN)

        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(
            JointState, 'joint_states/wheels', self.joint_state_callback, 10
        )

        self.timer_period   = 0.05
        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_feedback_time = now
        self.last_loop_time = now
        self.command_timeout_logged = False
        self.feedback_timeout_logged = False
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info(
            f'PID Motor Controller started — '
            f'Kp={kp}, Ki={ki}, Kd={kd}, '
            f'max_pwm={self.max_pwm}, max_linear={self.max_linear_mps}, '
            f'cmd_vel_topic={cmd_vel_topic}'
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.command_timeout_logged = False
        linear  = self.clamp(msg.linear.x, -self.max_linear_mps, self.max_linear_mps)
        angular = msg.angular.z
        self.target_left_mps  = (linear - angular * (self.track_width_m / 2.0)) \
                                 * self.left_motor_direction
        self.target_right_mps = (linear + angular * (self.track_width_m / 2.0)) \
                                 * self.right_motor_direction

    def joint_state_callback(self, msg: JointState):
        if len(msg.velocity) < 2:
            return
        try:
            left_idx  = msg.name.index('left_wheel')
            right_idx = msg.name.index('right_wheel')
            self.actual_left_mps = (
                msg.velocity[left_idx]
                * self.wheel_radius_m
                * self.left_feedback_direction
            )
            self.actual_right_mps = (
                msg.velocity[right_idx]
                * self.wheel_radius_m
                * self.right_feedback_direction
            )
#            if abs(self.actual_right_mps) > 0 or abs(self.actual_left_mps) > 0:
#                print(f'{self.actual_left_mps} {self.actual_right_mps}')
            self.last_feedback_time = self.get_clock().now()
            self.feedback_received = True
            self.feedback_timeout_logged = False
        except ValueError:
            pass

    def control_loop(self):
        now = self.get_clock().now()
        dt  = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        # Safety timeouts stop immediately because old commands/feedback are unsafe.
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.command_timeout_sec:
            if not self.command_timeout_logged:
                self.get_logger().warn('Command timeout; stopping motors')
                self.command_timeout_logged = True
            self.stop_motors()
            return

        feedback_elapsed = (now - self.last_feedback_time).nanoseconds / 1e9
        target_active = (
            abs(self.target_left_mps) >= self.deadband
            or abs(self.target_right_mps) >= self.deadband
        )
        if target_active and (
            not self.feedback_received
            or feedback_elapsed > self.feedback_timeout_sec
        ):
            if not self.feedback_timeout_logged:
                self.get_logger().warn('Encoder feedback timeout; stopping motors')
                self.feedback_timeout_logged = True
            self.stop_motors()
            return

        max_step = self.pwm_slew_per_sec * self.timer_period

        # When target is zero: slew smoothly to zero, don't hard-cut
        if abs(self.target_left_mps) < self.deadband and \
           abs(self.target_right_mps) < self.deadband:
            self.left_pid.reset()
            self.right_pid.reset()
            # Slew toward zero — smooth deceleration
            self.applied_left_pwm  = self.slew_toward(
                self.applied_left_pwm,  0.0, max_step
            )
            self.applied_right_pwm = self.slew_toward(
                self.applied_right_pwm, 0.0, max_step
            )
            self.set_left_motor(self.applied_left_pwm)
            self.set_right_motor(self.applied_right_pwm)
            return

        # PID computes target PWM from speed error
        left_error  = self.target_left_mps  - self.actual_left_mps
        right_error = self.target_right_mps - self.actual_right_mps

        left_target_pwm  = self.clamp(
            self.applied_left_pwm  + self.left_pid.compute(left_error,  dt),
            -self.max_pwm, self.max_pwm
        )
        right_target_pwm = self.clamp(
            self.applied_right_pwm + self.right_pid.compute(right_error, dt),
            -self.max_pwm, self.max_pwm
        )

        # Slew-limit for smooth acceleration
        self.applied_left_pwm  = self.slew_toward(
            self.applied_left_pwm,  left_target_pwm,  max_step
        )
        self.applied_right_pwm = self.slew_toward(
            self.applied_right_pwm, right_target_pwm, max_step
        )

        self.set_left_motor(self.enforce_min_pwm(self.applied_left_pwm))
        self.set_right_motor(self.enforce_min_pwm(self.applied_right_pwm))

        self.get_logger().debug(
            f'L: target={self.target_left_mps:.2f} actual={self.actual_left_mps:.2f} '
            f'err={left_error:.2f} pwm={self.applied_left_pwm:.2f} | '
            f'R: target={self.target_right_mps:.2f} actual={self.actual_right_mps:.2f} '
            f'err={right_error:.2f} pwm={self.applied_right_pwm:.2f}'
        )

    def slew_toward(self, current, target, max_step):
        delta      = self.clamp(target - current, -max_step, max_step)
        next_value = current + delta
        if abs(target) < self.deadband and abs(next_value) < self.deadband:
            return 0.0
        return next_value

    def enforce_min_pwm(self, pwm):
        if abs(pwm) < self.deadband:
            return 0.0
        if 0 < abs(pwm) < self.min_effective_pwm:
            return self.min_effective_pwm if pwm > 0 else -self.min_effective_pwm
        return pwm

    def stop_motors(self):
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.actual_left_mps = 0.0
        self.actual_right_mps = 0.0
        self.applied_left_pwm = 0.0
        self.applied_right_pwm = 0.0
        self.left_pid.reset()
        self.right_pid.reset()
        self.set_left_motor(0.0)
        self.set_right_motor(0.0)

    def set_left_motor(self, speed):
        speed = self.clamp(speed, -self.max_pwm, self.max_pwm)
        if speed > 0:
            self.ina1.off();  self.inb1.on(); self.pwm1.value = speed
        elif speed < 0:
            self.ina1.on(); self.inb1.off();  self.pwm1.value = abs(speed)
        else:
            self.ina1.off(); self.inb1.off(); self.pwm1.value = 0.0

    def set_right_motor(self, speed):
        speed = self.clamp(speed, -self.max_pwm, self.max_pwm)
        if speed > 0:
            self.ina2.off(); self.inb2.on();  self.pwm2.value = speed
        elif speed < 0:
            self.ina2.on();  self.inb2.off(); self.pwm2.value = abs(speed)
        else:
            self.ina2.off(); self.inb2.off(); self.pwm2.value = 0.0

    def clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def destroy_node(self):
        self.set_left_motor(0.0)
        self.set_right_motor(0.0)
        self.pwm1.close(); self.pwm2.close()
        self.ina1.close(); self.inb1.close()
        self.ina2.close(); self.inb2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
