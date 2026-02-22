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


def test_launch_file_importable():
    """测试 Launch 文件能否正常导入"""
    import importlib.util
    import os
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'first_launch.py'
    )
    spec = importlib.util.spec_from_file_location('first_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert hasattr(module, 'generate_launch_description')


def test_launch_description_not_empty():
    """测试 LaunchDescription 不为空"""
    import importlib.util
    import os
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'first_launch.py'
    )
    spec = importlib.util.spec_from_file_location('first_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    assert len(ld.entities) > 0, 'LaunchDescription 不能为空'


def test_node_can_publish():
    """测试节点能否正常发布消息"""
    node = Node('test_first_launch')
    pub = node.create_publisher(String, 'hello_topic', 10)
    msg = String()
    msg.data = 'test message'
    pub.publish(msg)

    topic_names = [t[0] for t in node.get_topic_names_and_types()]
    assert '/hello_topic' in topic_names
    node.destroy_node()
