#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher_node')
       # Initialze Publisher with the "/Integer" topic
        self.publisher = self.create_publisher(Int32, '/Integer', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
       # Assign the msg variable to i
        msg.data = self.i
       # Publish the msg
        self.publisher.publish(msg)
       # Increment the i
        self.i = self.i + 1


def main(args=None):
    rclpy.init(args=args)
    Publisher_node = Publisher()
    rclpy.spin(Publisher_node)
    Publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
