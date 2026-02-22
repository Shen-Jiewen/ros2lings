#!/usr/bin/env python3
import importlib.util
import os
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ros2lings_interfaces.srv import AddTwoInts


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def _load_student_module():
    """Load the student's service_py module using importlib."""
    src_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'src', 'service_py.py')
    spec = importlib.util.spec_from_file_location('service_py', src_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_student_module_has_server_class():
    """Student module must define AddTwoIntsServer class."""
    mod = _load_student_module()
    assert hasattr(mod, 'AddTwoIntsServer'), \
        'Student module must define AddTwoIntsServer class'


def test_student_server_is_a_node():
    """AddTwoIntsServer must inherit from rclpy.node.Node."""
    mod = _load_student_module()
    server = mod.AddTwoIntsServer()
    assert isinstance(server, Node), \
        'AddTwoIntsServer must inherit from Node'
    assert server.get_name() == 'add_two_ints_server'
    server.destroy_node()


def test_student_server_creates_service():
    """Student's server should create an 'add_two_ints' service."""
    mod = _load_student_module()
    server = mod.AddTwoIntsServer()

    client_node = Node('test_client')
    client = client_node.create_client(AddTwoInts, 'add_two_ints')
    assert client.wait_for_service(timeout_sec=2.0), \
        "Student's service 'add_two_ints' should be available"

    client_node.destroy_node()
    server.destroy_node()


def test_student_server_computes_correct_sum():
    """Student's handle_add callback should compute a + b correctly."""
    mod = _load_student_module()
    server = mod.AddTwoIntsServer()

    client_node = Node('test_compute_client')
    client = client_node.create_client(AddTwoInts, 'add_two_ints')
    assert client.wait_for_service(timeout_sec=2.0)

    request = AddTwoInts.Request()
    request.a = 10
    request.b = 20
    future = client.call_async(request)

    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    executor.spin_until_future_complete(future, timeout_sec=2.0)

    assert future.done(), 'Request should complete'
    result = future.result()
    assert result.sum == 30, '10 + 20 should equal 30'

    client_node.destroy_node()
    server.destroy_node()


def test_student_server_handles_negative_numbers():
    """Student's service should handle negative numbers."""
    mod = _load_student_module()
    server = mod.AddTwoIntsServer()

    client_node = Node('test_negative_client')
    client = client_node.create_client(AddTwoInts, 'add_two_ints')
    assert client.wait_for_service(timeout_sec=2.0)

    request = AddTwoInts.Request()
    request.a = -5
    request.b = 3
    future = client.call_async(request)

    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    executor.spin_until_future_complete(future, timeout_sec=2.0)

    assert future.done()
    result = future.result()
    assert result.sum == -2, '-5 + 3 should equal -2'

    client_node.destroy_node()
    server.destroy_node()
