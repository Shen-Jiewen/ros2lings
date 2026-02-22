#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    talker = Node(
        package='ros2lings_29_multi_node_launch',
        executable='talker_node.py',
        name='talker',
        output='screen',
    )

    listener = Node(
        package='ros2lings_29_multi_node_launch',
        executable='listener_node.py',
        name='listener',
        output='screen',
    )

    return LaunchDescription([talker, listener])
