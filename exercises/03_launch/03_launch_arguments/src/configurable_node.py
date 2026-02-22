#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ConfigurableNode(Node):

    def __init__(self):
        super().__init__('configurable_node')

        # 声明参数 'topic_name'，默认值为 'hello'
        self.declare_parameter('topic_name', 'hello')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.get_logger().info(f'Publishing to topic: {topic_name}')

        self.count_ = 0
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from configurable node! Count: {self.count_}'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)
        self.count_ += 1


def main():
    rclpy.init()
    node = ConfigurableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
