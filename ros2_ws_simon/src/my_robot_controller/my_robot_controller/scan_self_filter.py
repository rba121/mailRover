import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSelfFilterNode(Node):
    def __init__(self):
        super().__init__('scan_self_filter')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_filtered')
        self.declare_parameter('enabled', True)
        self.declare_parameter('laser_x_offset_m', 0.0)
        self.declare_parameter('laser_y_offset_m', 0.0)
        self.declare_parameter('self_filter_x_min_m', -0.25)
        self.declare_parameter('self_filter_x_max_m', 0.25)
        self.declare_parameter('self_filter_y_min_m', -0.25)
        self.declare_parameter('self_filter_y_max_m', 0.25)

        input_topic = self.get_parameter('input_scan_topic').value
        output_topic = self.get_parameter('output_scan_topic').value
        self.enabled = bool(self.get_parameter('enabled').value)
        self.laser_x_offset_m = float(self.get_parameter('laser_x_offset_m').value)
        self.laser_y_offset_m = float(self.get_parameter('laser_y_offset_m').value)
        self.self_filter_x_min_m = float(
            self.get_parameter('self_filter_x_min_m').value
        )
        self.self_filter_x_max_m = float(
            self.get_parameter('self_filter_x_max_m').value
        )
        self.self_filter_y_min_m = float(
            self.get_parameter('self_filter_y_min_m').value
        )
        self.self_filter_y_max_m = float(
            self.get_parameter('self_filter_y_max_m').value
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Scan self-filter publishing {input_topic} -> {output_topic}; '
            f'mask x=[{self.self_filter_x_min_m:.2f}, '
            f'{self.self_filter_x_max_m:.2f}], '
            f'y=[{self.self_filter_y_min_m:.2f}, '
            f'{self.self_filter_y_max_m:.2f}]'
        )

    def scan_callback(self, msg):
        if not self.enabled:
#            msg.header.stamp = self.get_clock().now().to_msg()
            self.scan_pub.publish(msg)
            return

        filtered = LaserScan()
        filtered.header = msg.header
#        filtered.header.stamp = self.get_clock().now().to_msg()
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = list(msg.ranges)
        filtered.intensities = list(msg.intensities)

        angle = msg.angle_min
        for index, reading in enumerate(filtered.ranges):
            if self.is_self_hit(reading, angle, msg.range_min, msg.range_max):
                filtered.ranges[index] = math.inf
            angle += msg.angle_increment

        self.scan_pub.publish(filtered)

    def is_self_hit(self, reading, angle, range_min, range_max):
        if not math.isfinite(reading) or reading < range_min or reading > range_max:
            return False

        base_x = self.laser_x_offset_m + (reading * math.cos(angle))
        base_y = self.laser_y_offset_m + (reading * math.sin(angle))

        return (
            self.self_filter_x_min_m <= base_x <= self.self_filter_x_max_m
            and self.self_filter_y_min_m <= base_y <= self.self_filter_y_max_m
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
