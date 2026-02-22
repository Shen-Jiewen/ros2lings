#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParamDemoNode(Node):

    def __init__(self):
        super().__init__('param_demo_node')

        # 先声明参数，再获取参数值
        self.declare_parameter('robot_name', 'default_bot')
        name = self.get_parameter('robot_name').value

        # max_count 的默认值为整数
        self.declare_parameter('max_count', 10)
        count = self.get_parameter('max_count').value

        # 声明并获取 timer_period 参数
        self.declare_parameter('timer_period', 1.0)
        period = self.get_parameter('timer_period').value

        self.robot_name_ = name
        self.max_count_ = count
        self.timer_period_ = period

        self.count_ = 0
        self.publisher_ = self.create_publisher(String, 'param_topic', 10)
        self.timer_ = None

        self.get_logger().info(f'robot_name: {self.robot_name_}')
        self.get_logger().info(f'max_count: {self.max_count_}')
        self.get_logger().info(f'timer_period: {self.timer_period_}')

    def start(self):
        """启动定时器，开始发布消息"""
        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

    def timer_callback(self):
        if self.count_ < self.max_count_:
            msg = String()
            msg.data = f'{self.robot_name_}: count {self.count_}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.count_ += 1

    def get_robot_name(self):
        return self.robot_name_

    def get_max_count(self):
        return self.max_count_


def main():
    rclpy.init()
    node = ParamDemoNode()
    node.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
