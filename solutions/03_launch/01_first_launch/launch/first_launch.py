#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='ros2lings_28_first_launch',
        executable='first_launch_node.py',
        name='first_launch_node',
        output='screen',
    )

    return LaunchDescription([node])
