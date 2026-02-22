#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='是否使用仿真模式'
    )

    sim_node = Node(
        package='ros2lings_34_conditional_launch',
        executable='sim_node.py',
        name='sim_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    real_node = Node(
        package='ros2lings_34_conditional_launch',
        executable='real_node.py',
        name='real_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
    )

    return LaunchDescription([
        use_sim_arg,
        sim_node,
        real_node,
    ])
