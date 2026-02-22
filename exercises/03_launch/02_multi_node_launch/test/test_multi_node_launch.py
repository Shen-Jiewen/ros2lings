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


def test_launch_description_has_two_entities():
    """测试 LaunchDescription 中有两个节点"""
    import importlib.util
    import os
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'multi_node_launch.py'
    )
    spec = importlib.util.spec_from_file_location('multi_node_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    assert len(ld.entities) == 2, \
        f'LaunchDescription 应该有 2 个节点，实际有 {len(ld.entities)} 个'


def test_talker_can_publish():
    """测试 talker 节点能否发布消息"""
    node = Node('test_talker')
    pub = node.create_publisher(String, 'chatter', 10)
    msg = String()
    msg.data = 'test from talker'
    pub.publish(msg)

    topic_names = [t[0] for t in node.get_topic_names_and_types()]
    assert '/chatter' in topic_names
    node.destroy_node()


def test_listener_can_subscribe():
    """测试 listener 节点能否订阅消息"""
    received = [False]

    node = Node('test_listener')
    pub = node.create_publisher(String, 'chatter', 10)

    def callback(msg):
        received[0] = True

    node.create_subscription(String, 'chatter', callback, 10)

    msg = String()
    msg.data = 'test message'
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)

    assert received[0], '订阅者未收到消息'
    node.destroy_node()
