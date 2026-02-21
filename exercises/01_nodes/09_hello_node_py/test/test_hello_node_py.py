#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_can_create_node():
    """测试能否成功创建节点"""
    node = Node('test_hello_node_py')
    assert node is not None
    node.destroy_node()


def test_node_has_correct_name():
    """测试节点名称是否正确"""
    node = Node('test_hello_node_py')
    assert node.get_name() == 'test_hello_node_py'
    node.destroy_node()


def test_can_spin_once():
    """测试节点能否执行一次 spin"""
    node = Node('test_spin_node_py')
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
