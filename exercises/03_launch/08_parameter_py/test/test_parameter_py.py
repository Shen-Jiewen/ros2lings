#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def _create_param_node():
    """创建一个正确配置参数的节点用于测试"""
    import importlib.util
    import os
    src_file = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'parameter_py.py'
    )
    spec = importlib.util.spec_from_file_location('parameter_py', src_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_node_can_be_created():
    """测试节点能否正常创建"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    assert node is not None
    node.destroy_node()


def test_robot_name_declared_before_get():
    """测试 robot_name 参数声明在获取之前"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    assert node.has_parameter('robot_name'), '节点应声明 robot_name 参数'
    node.destroy_node()


def test_robot_name_default_value():
    """测试 robot_name 参数默认值"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    value = node.get_parameter('robot_name').get_parameter_value().string_value
    assert value == 'default_bot', f'robot_name 默认值应为 "default_bot"，实际为 "{value}"'
    node.destroy_node()


def test_max_count_is_integer():
    """测试 max_count 参数类型为整数"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    param = node.get_parameter('max_count')
    from rcl_interfaces.msg import ParameterType
    assert param.type_ == ParameterType.PARAMETER_INTEGER, \
        f'max_count 应为整数类型，实际为 {param.type_}'
    node.destroy_node()


def test_max_count_default_value():
    """测试 max_count 参数默认值"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    value = node.get_parameter('max_count').get_parameter_value().integer_value
    assert value == 10, f'max_count 默认值应为 10，实际为 {value}'
    node.destroy_node()


def test_timer_period_declared():
    """测试 timer_period 参数已声明"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    assert node.has_parameter('timer_period'), '节点应声明 timer_period 参数'
    node.destroy_node()


def test_timer_period_is_double():
    """测试 timer_period 参数类型为浮点数"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    param = node.get_parameter('timer_period')
    from rcl_interfaces.msg import ParameterType
    assert param.type_ == ParameterType.PARAMETER_DOUBLE, \
        f'timer_period 应为浮点数类型，实际为 {param.type_}'
    node.destroy_node()


def test_timer_period_default_value():
    """测试 timer_period 参数默认值"""
    module = _create_param_node()
    node = module.ParamDemoNode()
    value = node.get_parameter('timer_period').get_parameter_value().double_value
    assert value == 1.0, f'timer_period 默认值应为 1.0，实际为 {value}'
    node.destroy_node()
