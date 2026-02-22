#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FirstLaunchNode(Node):

    def __init__(self):
        super().__init__('first_launch_node')
        self.count_ = 0
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from launch! Count: {self.count_}'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)
        self.count_ += 1


def main():
    rclpy.init()
    node = FirstLaunchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
