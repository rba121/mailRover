import serial
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import math
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

ROOM_COORDINATES = {
    # destination: (x, y, yaw_radians) in the map frame
    "room1": (2.33, 28.2, 0.0),
    "room2": (4.83, 34.1, 0.0),
}


class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')
        self.get_logger().info('UART bridge node started')

        self.active_task_id = None
        self.active_destination = None
        self.cancel_requested = False
        self.active_goal_handle = None
        self.nav_goal_in_progress = False

        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
            self.get_logger().info("Opened UART on /dev/ttyAMA0 @ 115200")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART: {e}")
            raise

        self.timer = self.create_timer(0.1, self.read_uart)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def read_uart(self):
        try:
            if not self.ser or not self.ser.is_open:
                return

            if not self.ser.in_waiting:
                return

            msg = self.ser.readline().decode(errors='ignore').strip()
            if not msg:
                return

            self.get_logger().info(f"UART RX: {msg}")
            parts = [p.strip() for p in msg.split('|')]

            if not parts:
                return

            if len(parts) == 4 and parts[0] == "TASK_CREATE":
                task_id = parts[1]
                destination = parts[2]
                drawer_id = parts[3]

                if not task_id:
                    self.get_logger().warn("TASK_CREATE received with empty task_id")
                    return

                if self.active_task_id is not None and self.active_task_id != task_id:
                    self.get_logger().warn(
                        f"Busy with task {self.active_task_id}; rejecting new task {task_id}"
                    )
                    self.send_status(task_id, "BLOCKED")
                    return

                if self.active_task_id == task_id and self.nav_goal_in_progress:
                    self.get_logger().warn(f"Duplicate TASK_CREATE ignored for active task {task_id}")
                    return

                self.active_task_id = task_id
                self.active_destination = destination
                self.cancel_requested = False

                self.get_logger().info(
                    f"Task received from BBG: task_id={task_id}, destination={destination}, drawer_id={drawer_id}"
                )
                self.handle_task_create(task_id, destination, drawer_id)

            elif len(parts) == 2 and parts[0] == "TASK_CANCEL":
                task_id = parts[1]

                if not task_id:
                    self.get_logger().warn("TASK_CANCEL received with empty task_id")
                    return

                self.get_logger().info(f"Task cancel received: {task_id}")
                self.handle_task_cancel(task_id)

            elif len(parts) == 2 and parts[0] == "DELIVERY_COMPLETE":
                task_id = parts[1]

                if not task_id:
                    self.get_logger().warn("DELIVERY_COMPLETE received with empty task_id")
                    return

                self.get_logger().info(f"Delivery complete received: {task_id}")
                self.handle_delivery_complete(task_id)

            else:
                self.get_logger().warn(f"Unknown or malformed UART message: {msg}")

        except Exception as e:
            self.get_logger().error(f"UART read error: {e}")

    def handle_task_create(self, task_id, destination, drawer_id):
        self.get_logger().info(
            f"Handling task creation: task_id={task_id}, destination={destination}, drawer_id={drawer_id}"
        )

        sent = self.send_status(task_id, "MOVING")
        if not sent:
            self.get_logger().error(f"Failed to send MOVING for task {task_id}")
            return

        if destination not in ROOM_COORDINATES:
            self.get_logger().warn(f"Unknown destination: {destination}")
            self.send_status(task_id, "BLOCKED")
            self.clear_active_task(task_id)
            return

        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 navigate_to_pose action server is not available")
            self.send_status(task_id, "BLOCKED")
            self.clear_active_task(task_id)
            return

        target_x, target_y, target_yaw = ROOM_COORDINATES[destination]
        self.get_logger().info(
            f"Sending Nav2 goal for {destination}: x={target_x:.2f}, y={target_y:.2f}, yaw={target_yaw:.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.nav_goal_in_progress = True
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self.nav_goal_response_callback(future, task_id)
        )

    def nav_goal_response_callback(self, future, task_id):
        if self.active_task_id != task_id:
            self.get_logger().info(f"Ignoring stale Nav2 goal response for task {task_id}")
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send Nav2 goal for task {task_id}: {e}")
            self.send_status(task_id, "BLOCKED")
            self.clear_active_task(task_id)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"Nav2 rejected task {task_id}")
            self.send_status(task_id, "BLOCKED")
            self.clear_active_task(task_id)
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info(f"Nav2 accepted task {task_id}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.nav_result_callback(future, task_id)
        )

    def nav_result_callback(self, future, task_id):
        if self.active_task_id != task_id:
            self.get_logger().info(f"Ignoring stale Nav2 result for task {task_id}")
            return

        try:
            wrapped_result = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 result error for task {task_id}: {e}")
            self.send_status(task_id, "BLOCKED")
            self.clear_active_task(task_id)
            return

        if self.cancel_requested:
            self.get_logger().info(f"Task {task_id} was cancelled")
            self.clear_active_task(task_id)
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            sent = self.send_status(task_id, "ARRIVED")
            if sent:
                self.get_logger().info(f"ARRIVED for task {task_id}")
            else:
                self.get_logger().error(f"Failed to send ARRIVED for task {task_id}")
        else:
            self.get_logger().warn(
                f"Nav2 did not complete task {task_id}; status={wrapped_result.status}"
            )
            self.send_status(task_id, "BLOCKED")

        self.clear_active_task(task_id)

    def handle_task_cancel(self, task_id):
        self.get_logger().info(f"Handling task cancel: task_id={task_id}")

        self.cancel_requested = True
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()

        self.send_status(task_id, "CANCELED")
        self.clear_active_task(task_id)

    def handle_delivery_complete(self, task_id):
        self.get_logger().info(f"Handling delivery complete: task_id={task_id}")

        if self.active_task_id != task_id:
            self.get_logger().warn(f"DELIVERY_COMPLETE for non-active task: {task_id}")
        self.cancel_requested = False

        self.send_status(task_id, "IDLE")
        self.clear_active_task(task_id)

    def clear_active_task(self, task_id):
        if self.active_task_id == task_id:
            self.active_task_id = None
            self.active_destination = None
            self.active_goal_handle = None
            self.nav_goal_in_progress = False

    def send_status(self, task_id, status):
        try:
            if not self.ser or not self.ser.is_open:
                self.get_logger().error("UART send error: serial port not open")
                return False

            msg = f"STATUS|{task_id}|{status}\n"
            self.ser.write(msg.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f"UART TX: {msg.strip()}")
            return True

        except Exception as e:
            self.get_logger().error(f"UART send error: {e}")
            return False

    def destroy_node(self):
        try:
            self.cancel_requested = True
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()

            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("UART port closed")
        except Exception as e:
            self.get_logger().warn(f"Error while closing UART: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UART bridge stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
