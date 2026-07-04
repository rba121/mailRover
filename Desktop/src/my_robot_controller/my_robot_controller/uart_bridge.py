import serial
import rclpy
from rclpy.node import Node
import threading
import time
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

ROOM_COORDINATES = {
    "room1": (3.54, 28.7),
    "room2": (5.86, 33.06),
}

ARRIVAL_THRESHOLD = 0.8


class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')
        self.get_logger().info('UART bridge node started')

        self.active_task_id = None
        self.active_destination = None
        self.task_thread = None
        self.cancel_requested = False
        self.lock = threading.Lock()

        self.current_x = None
        self.current_y = None

        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
            self.get_logger().info("Opened UART on /dev/ttyAMA0 @ 115200")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART: {e}")
            raise

        self.timer = self.create_timer(0.1, self.read_uart)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

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

                with self.lock:
                    if self.active_task_id is not None and self.active_task_id != task_id:
                        self.get_logger().warn(
                            f"Busy with task {self.active_task_id}; rejecting new task {task_id}"
                        )
                        self.send_status(task_id, "BLOCKED")
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

        if self.task_thread and self.task_thread.is_alive():
            self.get_logger().warn("Previous task thread still running; ignoring duplicate/new create")
            return

        sent = self.send_status(task_id, "MOVING")
        if not sent:
            self.get_logger().error(f"Failed to send MOVING for task {task_id}")
            return

        self.task_thread = threading.Thread(
            target=self.monitor_arrival,
            args=(task_id, destination),
            daemon=True
        )
        self.task_thread.start()

    def monitor_arrival(self, task_id, destination):
        if destination not in ROOM_COORDINATES:
            self.get_logger().warn(f"Unknown destination: {destination}, sending ARRIVED anyway")
            self.send_status(task_id, "ARRIVED")
            return

        target_x, target_y = ROOM_COORDINATES[destination]
        self.get_logger().info(
            f"Monitoring arrival at {destination} ({target_x}, {target_y})"
        )

        while True:
            with self.lock:
                if self.cancel_requested or self.active_task_id != task_id:
                    self.get_logger().info(f"Task {task_id} cancelled during navigation")
                    return

            if self.current_x is not None and self.current_y is not None:
                dist = math.sqrt(
                    (self.current_x - target_x) ** 2 +
                    (self.current_y - target_y) ** 2
                )

                self.get_logger().info(
                    f"Distance to {destination}: {dist:.2f}m (threshold: {ARRIVAL_THRESHOLD}m)"
                )

                if dist < ARRIVAL_THRESHOLD:
                    sent = self.send_status(task_id, "ARRIVED")
                    if sent:
                        self.get_logger().info(f"ARRIVED at {destination} for task {task_id}")
                    else:
                        self.get_logger().error(f"Failed to send ARRIVED for task {task_id}")
                    return
            else:
                self.get_logger().warn("Waiting for pose data from /pose topic...")

            time.sleep(0.5)

    def handle_task_cancel(self, task_id):
        self.get_logger().info(f"Handling task cancel: task_id={task_id}")

        with self.lock:
            self.cancel_requested = True

        self.send_status(task_id, "CANCELED")

        with self.lock:
            if self.active_task_id == task_id:
                self.active_task_id = None
                self.active_destination = None

    def handle_delivery_complete(self, task_id):
        self.get_logger().info(f"Handling delivery complete: task_id={task_id}")

        with self.lock:
            if self.active_task_id != task_id:
                self.get_logger().warn(f"DELIVERY_COMPLETE for non-active task: {task_id}")
            self.cancel_requested = False

        self.send_status(task_id, "IDLE")

        with self.lock:
            if self.active_task_id == task_id:
                self.active_task_id = None
                self.active_destination = None

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
            with self.lock:
                self.cancel_requested = True

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
