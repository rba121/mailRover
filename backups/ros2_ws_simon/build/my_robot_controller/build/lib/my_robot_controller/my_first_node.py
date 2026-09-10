#!/usr/bin/env  python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")  #node name
        
        #self.get_logger().info("ROS")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello")    

def main(args=None):
    rclpy.init(args=args) #initialze ROS2
    node = MyNode() #Initialsze contructor 
    rclpy.spin(node)  #kept alive indefinitely ctrl+c to kill  (all callbacks)
    rclpy.shutdown()  #shutdown ROS2 communications

if __name__ == '__main__':
    main()