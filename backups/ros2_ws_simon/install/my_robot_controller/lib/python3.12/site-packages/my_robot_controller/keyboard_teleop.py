import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.60

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Keyboard teleop started')

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    print("Keyboard teleop")
    print("w = forward")
    print("s = backward")
    print("a = turn left")
    print("d = turn right")
    print("space = stop")
    print("q = quit")

    try:
        while True:
            key = get_key().lower()

            if key == 'w':
                node.send_cmd(LINEAR_SPEED, 0.0)
                print("forward")

            elif key == 's':
                node.send_cmd(-LINEAR_SPEED, 0.0)
                print("backward")

            elif key == 'a':
                node.send_cmd(0.0, ANGULAR_SPEED)
                print("left")

            elif key == 'd':
                node.send_cmd(0.0, -ANGULAR_SPEED)
                print("right")

            elif key == ' ':
                node.send_cmd(0.0, 0.0)
                print("stop")

            elif key == 'q':
                node.send_cmd(0.0, 0.0)
                print("quit")
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.send_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
