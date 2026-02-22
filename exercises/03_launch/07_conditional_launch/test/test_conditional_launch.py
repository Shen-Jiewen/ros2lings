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
        os.path.dirname(__file__), '..', 'launch', 'conditional_launch.py'
    )
    spec = importlib.util.spec_from_file_location('conditional_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_file_importable():
    """测试 Launch 文件能否正常导入"""
    module = _load_launch_module()
    assert hasattr(module, 'generate_launch_description')


def test_launch_has_declare_launch_argument():
    """测试 LaunchDescription 中包含 DeclareLaunchArgument"""
    from launch.actions import DeclareLaunchArgument
    module = _load_launch_module()
    ld = module.generate_launch_description()

    has_declare = any(
        isinstance(entity, DeclareLaunchArgument)
        for entity in ld.entities
    )
    assert has_declare, 'LaunchDescription 中缺少 DeclareLaunchArgument'


def test_launch_has_conditional_nodes():
    """测试 LaunchDescription 中包含带条件的节点"""
    from launch_ros.actions import Node as LaunchNode
    module = _load_launch_module()
    ld = module.generate_launch_description()

    nodes = [e for e in ld.entities if isinstance(e, LaunchNode)]
    assert len(nodes) >= 2, '应该有至少 2 个节点（sim_node 和 real_node）'

    has_condition = any(
        n.condition is not None
        for n in nodes
    )
    assert has_condition, '至少一个节点应该有条件（IfCondition 或 UnlessCondition）'


def test_launch_description_not_empty():
    """测试 LaunchDescription 不为空"""
    module = _load_launch_module()
    ld = module.generate_launch_description()
    assert len(ld.entities) > 0, 'LaunchDescription 不能为空'


def test_sim_node_can_be_created():
    """测试仿真节点能正常创建"""
    node = Node('test_sim_node')
    assert node.get_name() == 'test_sim_node'
    node.destroy_node()


def test_real_node_can_be_created():
    """测试真实硬件节点能正常创建"""
    node = Node('test_real_node')
    assert node.get_name() == 'test_real_node'
    node.destroy_node()
