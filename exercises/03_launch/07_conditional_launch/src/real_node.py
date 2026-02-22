#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class RealNode(Node):

    def __init__(self):
        super().__init__('real_node')
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Running in real hardware mode')


def main():
    rclpy.init()
    node = RealNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
