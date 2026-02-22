#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ConfigurableNode(Node):

    def __init__(self):
        super().__init__('configurable_node')

        # 声明参数（带默认值）
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 0.0)
        self.declare_parameter('enable_logging', False)

        # 获取参数值
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.enable_logging = self.get_parameter('enable_logging').get_parameter_value().bool_value

        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.get_logger().info(f'max_speed: {self.max_speed}')
        self.get_logger().info(f'enable_logging: {self.enable_logging}')

        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.enable_logging:
            self.get_logger().info(
                f'{self.robot_name} running at speed {self.max_speed}')


def main():
    rclpy.init()
    node = ConfigurableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
