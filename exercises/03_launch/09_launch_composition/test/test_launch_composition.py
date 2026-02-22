#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def _load_launch_module():
    import importlib.util
    import os
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'composition_launch.py'
    )
    spec = importlib.util.spec_from_file_location('composition_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_file_importable():
    """测试 Launch 文件能否正常导入"""
    module = _load_launch_module()
    assert hasattr(module, 'generate_launch_description')


def test_launch_description_not_empty():
    """测试 LaunchDescription 不为空"""
    module = _load_launch_module()
    ld = module.generate_launch_description()
    assert len(ld.entities) > 0, 'LaunchDescription 不能为空'


def test_launch_has_composable_node_container():
    """测试 LaunchDescription 中包含 ComposableNodeContainer"""
    from launch_ros.actions import ComposableNodeContainer
    module = _load_launch_module()
    ld = module.generate_launch_description()

    has_container = any(
        isinstance(entity, ComposableNodeContainer)
        for entity in ld.entities
    )
    assert has_container, 'LaunchDescription 中缺少 ComposableNodeContainer'


def test_container_has_composable_nodes():
    """测试容器中包含组件节点"""
    from launch_ros.actions import ComposableNodeContainer
    module = _load_launch_module()
    ld = module.generate_launch_description()

    containers = [
        e for e in ld.entities
        if isinstance(e, ComposableNodeContainer)
    ]
    assert len(containers) > 0, '没有找到 ComposableNodeContainer'

    container = containers[0]
    # Access the private attribute (name-mangled)
    descriptions = container._ComposableNodeContainer__composable_node_descriptions
    assert len(descriptions) >= 2, \
        f'容器中应有至少 2 个组件节点，实际有 {len(descriptions)} 个'


def test_composable_nodes_have_correct_plugins():
    """测试组件节点使用了正确的 plugin 名称"""
    from launch_ros.actions import ComposableNodeContainer
    module = _load_launch_module()
    ld = module.generate_launch_description()

    containers = [
        e for e in ld.entities
        if isinstance(e, ComposableNodeContainer)
    ]
    container = containers[0]
    descriptions = container._ComposableNodeContainer__composable_node_descriptions

    plugin_names = []
    for desc in descriptions:
        # node_plugin is a list of TextSubstitution objects
        if hasattr(desc, 'node_plugin') and desc.node_plugin:
            plugin_names.append(desc.node_plugin[0].text)

    assert 'PublisherComponent' in plugin_names, \
        '组件节点中缺少 PublisherComponent plugin'
    assert 'SubscriberComponent' in plugin_names, \
        '组件节点中缺少 SubscriberComponent plugin'


def test_composition_topic_communication():
    """测试组件间的话题通信"""
    from std_msgs.msg import String
    node = Node('test_composition')
    pub = node.create_publisher(String, 'composition_topic', 10)
    msg = String()
    msg.data = 'test composition message'
    pub.publish(msg)

    topic_names = [t[0] for t in node.get_topic_names_and_types()]
    assert '/composition_topic' in topic_names
    node.destroy_node()
