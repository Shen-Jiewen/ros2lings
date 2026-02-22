#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


def main():
    rclpy.init()

    node = Node('hello_node_py')

    node.get_logger().info('Hello from Python ROS2 node!')

    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
