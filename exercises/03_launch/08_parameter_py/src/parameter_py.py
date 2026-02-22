#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: parameter_py
# 模块: 03 - Launch & Parameters
# 难度: ★★☆☆☆
#
# 学习目标:
#   掌握 Python 节点中参数的声明（declare_parameter）和获取（get_parameter），
#   理解参数的声明顺序和类型匹配规则。
#
# 说明:
#   下面的节点有 3 个错误需要你修复。
#
# 步骤:
#   1. 修复参数声明和获取的顺序 — 必须先 declare 再 get
#   2. 修复 max_count 的默认值类型 — 应为整数，而非字符串
#   3. 添加缺失的 timer_period 参数声明和获取
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParamDemoNode(Node):

    def __init__(self):
        super().__init__('param_demo_node')

        # BUG 1: 在 declare_parameter 之前调用了 get_parameter
        #         必须先声明参数，再获取参数值
        name = self.get_parameter('robot_name').value
        self.declare_parameter('robot_name', 'default_bot')

        # BUG 2: max_count 的默认值应为整数（如 10），不是字符串 'ten'
        self.declare_parameter('max_count', 'ten')

        # BUG 3: 缺少 timer_period 参数的声明和获取
        # self.declare_parameter('timer_period', 1.0)

        count = self.get_parameter('max_count').value
        # period = self.get_parameter('timer_period').value

        self.robot_name_ = name
        self.max_count_ = count
        # self.timer_period_ = period

        self.count_ = 0
        self.publisher_ = self.create_publisher(String, 'param_topic', 10)
        # BUG 3（续）: 添加 start() 方法创建 timer，使用 timer_period 参数值
        self.timer_ = None

        self.get_logger().info(f'robot_name: {self.robot_name_}')
        self.get_logger().info(f'max_count: {self.max_count_}')

    def start(self):
        """启动定时器，开始发布消息"""
        # self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)
        pass

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
