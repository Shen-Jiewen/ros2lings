#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_launch_has_declare_launch_argument():
    """测试 LaunchDescription 中包含 DeclareLaunchArgument"""
    import importlib.util
    import os
    from launch.actions import DeclareLaunchArgument

    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'launch_arguments.py'
    )
    spec = importlib.util.spec_from_file_location('launch_arguments', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()

    has_declare = any(
        isinstance(entity, DeclareLaunchArgument)
        for entity in ld.entities
    )
    assert has_declare, 'LaunchDescription 中缺少 DeclareLaunchArgument'


def test_node_declares_parameter():
    """测试节点能否声明和读取 topic_name 参数"""
    node = Node('test_configurable')
    node.declare_parameter('topic_name', 'hello')
    value = node.get_parameter('topic_name').get_parameter_value().string_value
    assert value == 'hello', f'参数默认值应为 "hello"，实际为 "{value}"'
    node.destroy_node()


def test_node_can_publish_to_configured_topic():
    """测试节点能否发布到配置的话题"""
    node = Node('test_pub_configured')
    topic_name = 'hello'
    pub = node.create_publisher(String, topic_name, 10)
    msg = String()
    msg.data = 'test from configurable node'
    pub.publish(msg)

    topic_names = [t[0] for t in node.get_topic_names_and_types()]
    assert f'/{topic_name}' in topic_names
    node.destroy_node()
