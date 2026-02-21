#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from ros2lings_interfaces.srv import AddTwoInts


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_can_create_service_server():
    """测试能否成功创建服务服务器"""
    node = Node('test_srv_create')
    srv = node.create_service(
        AddTwoInts, 'test_add_srv',
        lambda req, res: res)
    assert srv is not None
    node.destroy_node()


def test_can_create_service_client():
    """测试能否成功创建服务客户端"""
    node = Node('test_client_create')
    client = node.create_client(AddTwoInts, 'test_add_client')
    assert client is not None
    node.destroy_node()


def test_service_computes_correct_sum():
    """测试服务能正确计算两数之和"""
    node = Node('test_srv_compute')

    # 创建服务器
    def handle_add(request, response):
        response.sum = request.a + request.b
        return response

    srv = node.create_service(AddTwoInts, 'test_compute', handle_add)

    # 创建客户端
    client = node.create_client(AddTwoInts, 'test_compute')
    assert client.wait_for_service(timeout_sec=2.0), '服务应当可用'

    # 发送请求
    request = AddTwoInts.Request()
    request.a = 10
    request.b = 20
    future = client.call_async(request)

    # 等待结果
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    assert future.done(), '请求应当完成'

    result = future.result()
    assert result.sum == 30, '10 + 20 应当等于 30'
    node.destroy_node()


def test_service_handles_negative_numbers():
    """测试服务能处理负数"""
    node = Node('test_srv_negative')

    def handle_add(request, response):
        response.sum = request.a + request.b
        return response

    srv = node.create_service(AddTwoInts, 'test_negative', handle_add)
    client = node.create_client(AddTwoInts, 'test_negative')
    assert client.wait_for_service(timeout_sec=2.0)

    request = AddTwoInts.Request()
    request.a = -5
    request.b = 3
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    assert future.done()

    result = future.result()
    assert result.sum == -2, '-5 + 3 应当等于 -2'
    node.destroy_node()
