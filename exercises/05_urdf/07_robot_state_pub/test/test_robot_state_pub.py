#!/usr/bin/env python3
import pytest
import os
import importlib.util


def _load_launch_module():
    """加载 Launch 文件模块"""
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'display.launch.py'
    )
    spec = importlib.util.spec_from_file_location('display_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_file_importable():
    """测试 Launch 文件可被正常导入，且包含 generate_launch_description"""
    module = _load_launch_module()
    assert hasattr(module, 'generate_launch_description')


def test_launch_description_not_empty():
    """测试 LaunchDescription 不为空"""
    module = _load_launch_module()
    ld = module.generate_launch_description()
    assert len(ld.entities) > 0, 'LaunchDescription 不应为空'


def test_has_robot_state_publisher_node():
    """测试 LaunchDescription 中包含 Node action"""
    from launch_ros.actions import Node
    module = _load_launch_module()
    ld = module.generate_launch_description()
    has_rsp = any(
        isinstance(e, Node) for e in ld.entities
    )
    assert has_rsp, 'LaunchDescription 中应有 Node action'


def test_urdf_file_exists():
    """测试 urdf/robot.urdf 文件存在"""
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'robot.urdf'
    )
    assert os.path.exists(urdf_path), 'urdf/robot.urdf 文件应存在'


def test_urdf_is_valid():
    """测试 URDF 文件格式正确"""
    import xml.etree.ElementTree as ET
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'robot.urdf'
    )
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    assert root.tag == 'robot', 'URDF 根元素应为 robot'
