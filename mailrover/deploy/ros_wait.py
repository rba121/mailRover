#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosidl_runtime_py.utilities import get_message


def create_qos(kind: str):
    if kind == "sensor":
        return qos_profile_sensor_data

    if kind == "map":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def wait_for_topic(
    topic: str,
    message_type: str,
    timeout: float,
    qos_kind: str,
) -> int:
    rclpy.init()
    node = Node("mailrover_topic_readiness")
    received = False

    def callback(_message):
        nonlocal received
        received = True

    try:
        message_class = get_message(message_type)

        subscription = node.create_subscription(
            message_class,
            topic,
            callback,
            create_qos(qos_kind),
        )

        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

            if received:
                print(f"READY topic={topic}", flush=True)
                node.destroy_subscription(subscription)
                return 0

        print(
            f"TIMEOUT topic={topic} timeout={timeout}",
            file=sys.stderr,
            flush=True,
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def wait_for_action(action_name: str, timeout: float) -> int:
    rclpy.init()
    node = Node("mailrover_action_readiness")

    try:
        action_client = ActionClient(
            node,
            NavigateToPose,
            action_name,
        )

        if action_client.wait_for_server(timeout_sec=timeout):
            print(f"READY action={action_name}", flush=True)
            return 0

        print(
            f"TIMEOUT action={action_name} timeout={timeout}",
            file=sys.stderr,
            flush=True,
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(
        dest="mode",
        required=True,
    )

    topic_parser = subparsers.add_parser("topic")
    topic_parser.add_argument("--topic", required=True)
    topic_parser.add_argument("--type", required=True)
    topic_parser.add_argument("--timeout", type=float, required=True)
    topic_parser.add_argument(
        "--qos",
        choices=["sensor", "reliable", "map"],
        default="sensor",
    )

    action_parser = subparsers.add_parser("action")
    action_parser.add_argument(
        "--name",
        default="/navigate_to_pose",
    )
    action_parser.add_argument(
        "--timeout",
        type=float,
        required=True,
    )

    args = parser.parse_args()

    if args.mode == "topic":
        return wait_for_topic(
            args.topic,
            args.type,
            args.timeout,
            args.qos,
        )

    return wait_for_action(
        args.name,
        args.timeout,
    )


if __name__ == "__main__":
    raise SystemExit(main())
