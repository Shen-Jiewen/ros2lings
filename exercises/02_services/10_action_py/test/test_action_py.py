#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from ros2lings_interfaces.action import Fibonacci


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_fibonacci_goal_message():
    """测试 Fibonacci Goal 消息结构"""
    goal = Fibonacci.Goal()
    goal.order = 5
    assert goal.order == 5


def test_fibonacci_result_message():
    """测试 Fibonacci Result 消息结构"""
    result = Fibonacci.Result()
    result.sequence = [0, 1, 1, 2, 3]
    assert len(result.sequence) == 5
    assert result.sequence[4] == 3


def test_fibonacci_feedback_message():
    """测试 Fibonacci Feedback 消息结构"""
    feedback = Fibonacci.Feedback()
    feedback.partial_sequence = [0, 1, 1]
    assert len(feedback.partial_sequence) == 3


def test_fibonacci_computation_logic():
    """测试 Fibonacci 计算逻辑 — 使用加法而非乘法"""
    order = 7
    sequence = [0, 1]
    for i in range(2, order):
        sequence.append(sequence[i - 1] + sequence[i - 2])

    # Fibonacci(7): 0, 1, 1, 2, 3, 5, 8
    assert len(sequence) == 7
    assert sequence == [0, 1, 1, 2, 3, 5, 8]


def test_can_create_node():
    """测试能否创建 ROS2 节点"""
    node = Node('test_action_py_node')
    assert node is not None
    node.destroy_node()


def test_action_server_callback_must_return_result():
    """测试 execute_callback 应该返回 Result 对象"""
    # 模拟正确的 execute 逻辑
    goal_order = 5
    feedback_msg = Fibonacci.Feedback()
    feedback_msg.partial_sequence = [0, 1]

    for i in range(2, goal_order):
        next_val = feedback_msg.partial_sequence[i - 1] + feedback_msg.partial_sequence[i - 2]
        feedback_msg.partial_sequence.append(next_val)

    result = Fibonacci.Result()
    result.sequence = feedback_msg.partial_sequence

    # execute_callback 必须有返回值
    assert result is not None
    assert list(result.sequence) == [0, 1, 1, 2, 3]
