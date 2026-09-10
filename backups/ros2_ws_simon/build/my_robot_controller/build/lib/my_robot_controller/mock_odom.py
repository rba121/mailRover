#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math


class MockOdom(Node):

    def __init__(self):
        super().__init__('mock_odom')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        #Timer(20 Hz)
        self.timer = self.create_timer(0.05, self.update_odom)

        #State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Velocities
        self.vx = 0.0
        self.vth = 0.0

        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # ONLY store velocities
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        #Integrate motion
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        #TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)

        self.tf_broadcaster.sendTransform(t)

        #ODOM
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        self.last_time = current_time


def main():
    rclpy.init()
    node = MockOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
