#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yaml_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'params.yaml'
    )

    node = Node(
        package='ros2lings_33_yaml_config',
        executable='configurable_node.py',
        name='configurable_node',
        output='screen',
        parameters=[yaml_path],
    )

    return LaunchDescription([node])
