#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='hello',
        description='要发布的话题名称',
    )

    topic_config = LaunchConfiguration('topic_name')

    node = Node(
        package='ros2lings_30_launch_arguments',
        executable='configurable_node.py',
        name='configurable_node',
        output='screen',
        parameters=[{'topic_name': topic_config}],
    )

    return LaunchDescription([topic_arg, node])
