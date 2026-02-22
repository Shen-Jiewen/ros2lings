#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription_ = self.create_subscription(
            String, 'chatter', self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
