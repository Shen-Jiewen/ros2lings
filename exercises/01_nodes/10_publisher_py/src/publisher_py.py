#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: publisher_py
# 模块: 01 - Nodes & Topics
# 难度: ★☆☆☆☆
#
# 学习目标:
#   理解 rclpy 中 Publisher 的创建和 Timer 回调机制。
#
# 说明:
#   下面的代码尝试创建一个发布者节点，每秒向话题发布一条字符串消息。
#   但代码中有几个错误需要你修复。
#
# 步骤:
#   1. 修复 create_publisher 的调用——缺少 QoS 参数
#   2. 修复 Timer 的回调绑定——回调函数未正确绑定到实例方法
#   3. 补充消息发布调用
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherPy(Node):

    def __init__(self):
        super().__init__('publisher_py')
        self.count_ = 0

        # TODO: create_publisher 需要三个参数：消息类型、话题名称、QoS 深度
        # 提示: self.create_publisher(消息类型, '话题名', QoS深度)
        self.publisher_ = self.create_publisher(String, 'chatter')  # Bug: 缺少 QoS 参数

        # TODO: create_timer 的回调参数应该是 self.timer_callback 而不是函数名字符串
        # 提示: 回调需要是一个可调用对象
        self.timer_ = self.create_timer(0.5, 'timer_callback')  # Bug: 回调是字符串而非方法引用

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! Count: {self.count_}'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count_ += 1

        # TODO: 使用 self.publisher_ 发布 msg
        # 提示: self.publisher_.???


def main():
    rclpy.init()
    node = PublisherPy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
