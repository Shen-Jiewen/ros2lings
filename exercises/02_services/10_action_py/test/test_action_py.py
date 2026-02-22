#!/usr/bin/env python3
import importlib.util
import os
import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ros2lings_interfaces.action import Fibonacci


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def _load_student_module():
    """Load the student's action_py module using importlib."""
    src_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'src', 'action_py.py')
    spec = importlib.util.spec_from_file_location('action_py', src_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_student_module_has_server_class():
    """Student module must define FibonacciActionServer class."""
    mod = _load_student_module()
    assert hasattr(mod, 'FibonacciActionServer'), \
        'Student module must define FibonacciActionServer class'


def test_student_server_is_a_node():
    """FibonacciActionServer must inherit from Node."""
    mod = _load_student_module()
    server = mod.FibonacciActionServer()
    assert isinstance(server, Node), \
        'FibonacciActionServer must inherit from Node'
    assert server.get_name() == 'fibonacci_action_server_py'
    server.destroy_node()


def test_student_server_accepts_goal_and_returns_result():
    """Student's action server should accept a goal and return a correct result."""
    mod = _load_student_module()
    server = mod.FibonacciActionServer()

    client_node = Node('test_action_client')
    action_client = ActionClient(client_node, Fibonacci, 'fibonacci')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)

    assert action_client.wait_for_server(timeout_sec=2.0), \
        "Student's action server 'fibonacci' should be available"

    goal_msg = Fibonacci.Goal()
    goal_msg.order = 7

    send_goal_future = action_client.send_goal_async(goal_msg)
    executor.spin_until_future_complete(send_goal_future, timeout_sec=5.0)
    assert send_goal_future.done()
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted, 'Goal should be accepted'

    result_future = goal_handle.get_result_async()
    executor.spin_until_future_complete(result_future, timeout_sec=5.0)
    assert result_future.done()
    result = result_future.result().result

    # Fibonacci(7): 0, 1, 1, 2, 3, 5, 8
    expected = [0, 1, 1, 2, 3, 5, 8]
    assert list(result.sequence) == expected, \
        f'Expected {expected}, got {list(result.sequence)}'

    client_node.destroy_node()
    server.destroy_node()


def test_student_server_uses_addition_not_multiplication():
    """Student must fix the bug: Fibonacci uses addition, not multiplication."""
    mod = _load_student_module()
    server = mod.FibonacciActionServer()

    client_node = Node('test_action_add_client')
    action_client = ActionClient(client_node, Fibonacci, 'fibonacci')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)

    assert action_client.wait_for_server(timeout_sec=2.0)

    goal_msg = Fibonacci.Goal()
    goal_msg.order = 5

    send_goal_future = action_client.send_goal_async(goal_msg)
    executor.spin_until_future_complete(send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    result_future = goal_handle.get_result_async()
    executor.spin_until_future_complete(result_future, timeout_sec=5.0)
    result = result_future.result().result

    # Fibonacci(5): 0, 1, 1, 2, 3
    # If multiplication was used: 0, 1, 0, 0, 0 (wrong)
    expected = [0, 1, 1, 2, 3]
    assert list(result.sequence) == expected, \
        f'Expected {expected}, got {list(result.sequence)} (make sure to use + not *)'

    client_node.destroy_node()
    server.destroy_node()


def test_student_callback_returns_result():
    """Student's execute_callback must return a Result object."""
    mod = _load_student_module()
    server = mod.FibonacciActionServer()

    client_node = Node('test_return_client')
    action_client = ActionClient(client_node, Fibonacci, 'fibonacci')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)

    assert action_client.wait_for_server(timeout_sec=2.0)

    goal_msg = Fibonacci.Goal()
    goal_msg.order = 3

    send_goal_future = action_client.send_goal_async(goal_msg)
    executor.spin_until_future_complete(send_goal_future, timeout_sec=5.0)
    goal_handle = send_goal_future.result()
    assert goal_handle.accepted

    result_future = goal_handle.get_result_async()
    executor.spin_until_future_complete(result_future, timeout_sec=5.0)
    assert result_future.done(), \
        'execute_callback must return a Result object'

    result = result_future.result().result
    assert result is not None, 'Result must not be None (missing return statement?)'
    assert len(result.sequence) == 3

    client_node.destroy_node()
    server.destroy_node()
