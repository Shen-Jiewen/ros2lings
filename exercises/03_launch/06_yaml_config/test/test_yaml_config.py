#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node


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
        os.path.dirname(__file__), '..', 'launch', 'yaml_config.py'
    )
    spec = importlib.util.spec_from_file_location('yaml_config', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert hasattr(module, 'generate_launch_description')


def test_launch_description_not_empty():
    """测试 LaunchDescription 不为空"""
    import importlib.util
    import os
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'yaml_config.py'
    )
    spec = importlib.util.spec_from_file_location('yaml_config', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    assert len(ld.entities) > 0, 'LaunchDescription 不能为空'


def test_yaml_file_exists():
    """测试 YAML 配置文件存在"""
    import os
    yaml_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'params.yaml'
    )
    assert os.path.isfile(yaml_path), f'找不到配置文件: {yaml_path}'


def test_yaml_has_correct_parameters():
    """测试 YAML 文件包含正确的参数"""
    import os
    import yaml
    yaml_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'params.yaml'
    )
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    params = config['configurable_node']['ros__parameters']
    assert params['robot_name'] == 'ros2bot', '参数 robot_name 应为 "ros2bot"'
    assert params['max_speed'] == 1.5, '参数 max_speed 应为 1.5'
    assert params['enable_logging'] is True, '参数 enable_logging 应为 true'


def test_node_can_declare_parameters():
    """测试节点能否声明配置文件中的参数"""
    node = Node('test_configurable')
    node.declare_parameter('robot_name', 'default_robot')
    node.declare_parameter('max_speed', 0.0)
    node.declare_parameter('enable_logging', False)

    name_val = node.get_parameter('robot_name').get_parameter_value().string_value
    assert name_val == 'default_robot'

    speed_val = node.get_parameter('max_speed').get_parameter_value().double_value
    assert speed_val == 0.0

    log_val = node.get_parameter('enable_logging').get_parameter_value().bool_value
    assert log_val is False

    node.destroy_node()
