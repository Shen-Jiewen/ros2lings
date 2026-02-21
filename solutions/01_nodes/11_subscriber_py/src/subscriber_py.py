#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberPy(Node):

    def __init__(self):
        super().__init__('subscriber_py')

        self.subscription_ = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main():
    rclpy.init()
    node = SubscriberPy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
