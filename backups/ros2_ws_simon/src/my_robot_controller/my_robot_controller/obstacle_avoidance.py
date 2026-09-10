import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('cmd_vel_in', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_out', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic/front')
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('side_angle_deg', 85.0)
        self.declare_parameter('slow_distance', 0.75)
        self.declare_parameter('stop_distance', 0.38)
        self.declare_parameter('emergency_distance', 0.25)
        self.declare_parameter('blockage_wait_time', 5.0)
        self.declare_parameter('allow_turning_when_blocked', False)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('sensor_timeout', 0.75)

        self.front_angle = math.radians(
            self.get_parameter('front_angle_deg').value
        )
        self.side_angle = math.radians(
            self.get_parameter('side_angle_deg').value
        )
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.blockage_wait_time = float(
            self.get_parameter('blockage_wait_time').value
        )
        self.allow_turning_when_blocked = bool(
            self.get_parameter('allow_turning_when_blocked').value
        )
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)

        cmd_vel_in = self.get_parameter('cmd_vel_in').value
        cmd_vel_out = self.get_parameter('cmd_vel_out').value
        scan_topic = self.get_parameter('scan_topic').value
        ultrasonic_topic = self.get_parameter('ultrasonic_topic').value

        self.latest_cmd = Twist()
        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_scan_time = now
        self.last_ultrasonic_time = now
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.ultrasonic_min = math.inf
        self.scan_seen = False
        self.ultrasonic_seen = False
        self.blocked_since = None
        self.replan_logged = False

        self.cmd_sub = self.create_subscription(
            Twist,
            cmd_vel_in,
            self.cmd_vel_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10,
        )
        self.ultrasonic_sub = self.create_subscription(
            Range,
            ultrasonic_topic,
            self.ultrasonic_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_out, 10)
        self.timer = self.create_timer(0.05, self.publish_safe_cmd)

        self.get_logger().info(
            f'Obstacle avoidance filtering {cmd_vel_in} -> {cmd_vel_out} using '
            f'{scan_topic} and {ultrasonic_topic}'
        )

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def scan_callback(self, msg):
        front = []
        left = []
        right = []

        angle = msg.angle_min
        for reading in msg.ranges:
            if math.isfinite(reading) and msg.range_min <= reading <= msg.range_max:
                if abs(angle) <= self.front_angle:
                    front.append(reading)
                elif 0.0 < angle <= self.side_angle:
                    left.append(reading)
                elif -self.side_angle <= angle < 0.0:
                    right.append(reading)
            angle += msg.angle_increment

        self.front_min = min(front) if front else math.inf
        self.left_min = min(left) if left else math.inf
        self.right_min = min(right) if right else math.inf
        self.scan_seen = True
        self.last_scan_time = self.get_clock().now()

    def ultrasonic_callback(self, msg):
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            self.ultrasonic_min = msg.range
            self.ultrasonic_seen = True
            self.last_ultrasonic_time = self.get_clock().now()

    def get_front_obstacle_distance(self):
        now = self.get_clock().now()
        front_distances = []

        scan_age = (now - self.last_scan_time).nanoseconds / 1e9
        if self.scan_seen and scan_age <= self.sensor_timeout:
            front_distances.append(self.front_min)

        ultrasonic_age = (now - self.last_ultrasonic_time).nanoseconds / 1e9
        if self.ultrasonic_seen and ultrasonic_age <= self.sensor_timeout:
            front_distances.append(self.ultrasonic_min)

        return min(front_distances) if front_distances else math.inf

    def publish_safe_cmd(self):
        safe_cmd = Twist()
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.cmd_pub.publish(safe_cmd)
            return

        safe_cmd.linear.x = self.latest_cmd.linear.x
        safe_cmd.angular.z = self.latest_cmd.angular.z

        front_obstacle_distance = self.get_front_obstacle_distance()
        now = self.get_clock().now()

        if self.latest_cmd.linear.x > 0.0:
            if front_obstacle_distance <= self.stop_distance:
                if self.blocked_since is None:
                    self.blocked_since = now
                    self.replan_logged = False
                    self.get_logger().warn(
                        f'Obstacle detected at {front_obstacle_distance:.2f}m; stopping'
                    )

                blocked_time = (now - self.blocked_since).nanoseconds / 1e9
                safe_cmd.linear.x = 0.0

                if (
                    not self.allow_turning_when_blocked
                    or blocked_time < self.blockage_wait_time
                ):
                    safe_cmd.angular.z = 0.0
                elif not self.replan_logged:
                    self.get_logger().warn(
                        'Obstacle is still present; allowing Nav2 recovery/replanning '
                        'but blocking unsafe forward motion'
                    )
                    self.replan_logged = True

            elif front_obstacle_distance <= self.slow_distance:
                self.blocked_since = None
                self.replan_logged = False
                scale = max(
                    0.25,
                    (front_obstacle_distance - self.stop_distance)
                    / (self.slow_distance - self.stop_distance),
                )
                safe_cmd.linear.x *= scale
            else:
                self.blocked_since = None
                self.replan_logged = False
        else:
            if front_obstacle_distance > self.stop_distance:
                self.blocked_since = None
                self.replan_logged = False

        self.cmd_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
