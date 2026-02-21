#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherPy(Node):

    def __init__(self):
        super().__init__('publisher_py')
        self.count_ = 0

        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! Count: {self.count_}'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)
        self.count_ += 1


def main():
    rclpy.init()
    node = PublisherPy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
