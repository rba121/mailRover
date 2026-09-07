#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("mailrover_initial_pose_publisher")

    confirmed = False
    pose_sent = False

    def amcl_pose_callback(_message):
        nonlocal confirmed

        if pose_sent:
            confirmed = True

    qos = QoSProfile(depth=10)

    # Subscribe before publishing so the AMCL confirmation cannot be missed.
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        amcl_pose_callback,
        qos,
    )

    publisher = node.create_publisher(
        PoseWithCovarianceStamped,
        "/initialpose",
        qos,
    )

    try:
        subscriber_deadline = time.monotonic() + 20.0

        while (
            publisher.get_subscription_count() == 0
            and time.monotonic() < subscriber_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.2)

        if publisher.get_subscription_count() == 0:
            print(
                "ERROR: AMCL is not subscribed to /initialpose.",
                flush=True,
            )
            return 1

        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"

        # A zero timestamp asks TF to use the latest available transform.
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0

        message.pose.pose.position.x = args.x
        message.pose.pose.position.y = args.y
        message.pose.pose.position.z = 0.0

        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685

        deadline = time.monotonic() + args.timeout

        while time.monotonic() < deadline:
            pose_sent = True
            publisher.publish(message)

            confirmation_window = time.monotonic() + 2.0

            while time.monotonic() < confirmation_window:
                rclpy.spin_once(node, timeout_sec=0.2)

                if confirmed:
                    print(
                        "INITIAL_POSE_CONFIRMED "
                        f"x={args.x} y={args.y} yaw={args.yaw}",
                        flush=True,
                    )
                    return 0

        print(
            "ERROR: Initial pose was published, but /amcl_pose "
            "was not received before timeout.",
            flush=True,
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
