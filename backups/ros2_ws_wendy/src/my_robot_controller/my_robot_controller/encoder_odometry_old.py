import math
import threading

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from gpiozero import DigitalInputDevice
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class QuadratureEncoder:
    TRANSITIONS = {
        0b0001: 1,
        0b0111: 1,
        0b1110: 1,
        0b1000: 1,
        0b0010: -1,
        0b1011: -1,
        0b1101: -1,
        0b0100: -1,
    }

    def __init__(self, a_pin, b_pin, pull_up=False):
        self.a = DigitalInputDevice(a_pin, pull_up=pull_up)
        self.b = DigitalInputDevice(b_pin, pull_up=pull_up)
        self.lock = threading.Lock()
        self.count = 0
        self.last_state = self.read_state()

        self.a.when_activated = self.update
        self.a.when_deactivated = self.update
        self.b.when_activated = self.update
        self.b.when_deactivated = self.update

    def read_state(self):
        return (int(self.a.value) << 1) | int(self.b.value)

    def update(self):
        with self.lock:
            new_state = self.read_state()
            transition = (self.last_state << 2) | new_state
            self.count += self.TRANSITIONS.get(transition, 0)
            self.last_state = new_state

    def get_count(self):
        with self.lock:
            return self.count

    def close(self):
        self.a.close()
        self.b.close()


class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        self.declare_parameter('left_encoder_a_pin', 5)
        self.declare_parameter('left_encoder_b_pin', 6)
        self.declare_parameter('right_encoder_a_pin', 13)
        self.declare_parameter('right_encoder_b_pin', 19)
        self.declare_parameter('encoder_pull_up', False)
        self.declare_parameter('ticks_per_wheel_rev', 735.0)
        self.declare_parameter('wheel_radius_m', 0.075)
        self.declare_parameter('track_width_m', 0.4572)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('left_direction', 1.0)
        self.declare_parameter('right_direction', 1.0)

        left_a_pin = int(self.get_parameter('left_encoder_a_pin').value)
        left_b_pin = int(self.get_parameter('left_encoder_b_pin').value)
        right_a_pin = int(self.get_parameter('right_encoder_a_pin').value)
        right_b_pin = int(self.get_parameter('right_encoder_b_pin').value)
        pull_up = bool(self.get_parameter('encoder_pull_up').value)

        self.ticks_per_wheel_rev = float(
            self.get_parameter('ticks_per_wheel_rev').value
        )
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.track_width_m = float(self.get_parameter('track_width_m').value)
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.left_direction = float(self.get_parameter('left_direction').value)
        self.right_direction = float(self.get_parameter('right_direction').value)

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        timer_period = 1.0 / publish_rate_hz

        self.left_encoder = QuadratureEncoder(left_a_pin, left_b_pin, pull_up)
        self.right_encoder = QuadratureEncoder(right_a_pin, right_b_pin, pull_up)

        self.last_left_count = self.left_encoder.get_count()
        self.last_right_count = self.right_encoder.get_count()
        self.last_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states/wheels', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.timer = self.create_timer(timer_period, self.publish_odometry)

        self.get_logger().info(
            'Encoder odometry started: '
            f'L=GPIO{left_a_pin}/GPIO{left_b_pin}, '
            f'R=GPIO{right_a_pin}/GPIO{right_b_pin}, '
            f'ticks_per_wheel_rev={self.ticks_per_wheel_rev:.1f}, '
            f'publish_tf={self.publish_tf}'
        )

    def publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        left_count = self.left_encoder.get_count()
        right_count = self.right_encoder.get_count()

        delta_left_ticks = (left_count - self.last_left_count) * self.left_direction
        delta_right_ticks = (right_count - self.last_right_count) * self.right_direction

        self.last_left_count = left_count
        self.last_right_count = right_count
        self.last_time = now

        meters_per_tick = (
            2.0 * math.pi * self.wheel_radius_m / self.ticks_per_wheel_rev
        )
        delta_left = delta_left_ticks * meters_per_tick
        delta_right = delta_right_ticks * meters_per_tick

        delta_distance = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / self.track_width_m

        mid_theta = self.theta + (delta_theta / 2.0)
        self.x += delta_distance * math.cos(mid_theta)
        self.y += delta_distance * math.sin(mid_theta)
        self.theta = self.normalize_angle(self.theta + delta_theta)

        linear_velocity = delta_distance / dt
        angular_velocity = delta_theta / dt
        orientation = self.yaw_to_quaternion(self.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = orientation
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()
        joint_msg.name = ['left_wheel', 'right_wheel']
        joint_msg.position = [
            (left_count / self.ticks_per_wheel_rev) * 2.0 * math.pi,
            (right_count / self.ticks_per_wheel_rev) * 2.0 * math.pi,
        ]
        joint_msg.velocity = [
            delta_left / (self.wheel_radius_m * dt),
            delta_right / (self.wheel_radius_m * dt),
        ]
        self.joint_pub.publish(joint_msg)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header.stamp = now.to_msg()
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(transform)

    def yaw_to_quaternion(self, yaw):
        quat = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self):
        self.left_encoder.close()
        self.right_encoder.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
