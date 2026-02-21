#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: action_py
# 模块: 02 - Services & Actions
# 难度: ★★☆☆☆
#
# 学习目标:
#   理解 Python Action Server 的创建方式，包括构造函数参数顺序、
#   Fibonacci 计算逻辑和反馈发布方法。
#
# 说明:
#   下面的代码尝试创建一个 Python Fibonacci Action Server，
#   但代码中有四个错误。你需要找到并修复它们。
#
# 步骤:
#   1. 修复 ActionServer 构造函数的参数顺序
#   2. 修复 Fibonacci 计算（加法不是乘法）
#   3. 修复反馈发布的方法名
#   4. 添加 return result
#   5. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ros2lings_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server_py')

        # BUG 1: ActionServer 参数顺序错误 — 应该是 (node, type, name, callback)
        self._action_server = ActionServer(
            self,
            'fibonacci',
            Fibonacci,
            self.execute_callback
        )

        self.get_logger().info('Fibonacci Action Server (Python) started')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(2, goal_handle.request.order):
            # BUG 2: Fibonacci 应该用加法，不是乘法
            next_val = feedback_msg.partial_sequence[i - 1] * feedback_msg.partial_sequence[i - 2]
            feedback_msg.partial_sequence.append(next_val)

            # BUG 3: 方法名应该是 publish_feedback，不是 send_feedback
            goal_handle.send_feedback(feedback_msg)

            self.get_logger().info(
                'Feedback: {0}'.format(feedback_msg.partial_sequence))

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.get_logger().info('Goal succeeded')
        # BUG 4: 缺少 return result


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
