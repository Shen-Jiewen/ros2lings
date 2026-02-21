#!/usr/bin/env python3
# 参考答案 — action_py
# Python Fibonacci Action Server: 正确的构造参数、计算和反馈

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ros2lings_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server_py')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci Action Server (Python) started')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(2, goal_handle.request.order):
            next_val = feedback_msg.partial_sequence[i - 1] + feedback_msg.partial_sequence[i - 2]
            feedback_msg.partial_sequence.append(next_val)

            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                'Feedback: {0}'.format(feedback_msg.partial_sequence))

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.get_logger().info('Goal succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
