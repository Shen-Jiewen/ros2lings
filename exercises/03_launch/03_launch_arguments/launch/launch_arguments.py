#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: launch_arguments
# 模块: 03 - Launch & Parameters
# 难度: ★★☆☆☆
#
# 学习目标:
#   学会使用 DeclareLaunchArgument 和 LaunchConfiguration
#   在 Launch 文件中声明和传递参数。
#
# 说明:
#   下面的 Launch 文件需要你实现参数传递功能。
#   configurable_node.py 接受一个 'topic_name' 参数来决定发布话题。
#   你需要通过 Launch 参数把话题名传递给节点。
#
# 步骤:
#   1. 用 DeclareLaunchArgument 声明 'topic_name' 参数（默认值 'hello'）
#   2. 用 LaunchConfiguration 获取参数值
#   3. 把参数值通过 Node 的 parameters 传给节点
#   4. 实现完成后，删除文件顶部的 "# I AM NOT DONE"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO 1: 用 DeclareLaunchArgument 声明参数 'topic_name'
    #         默认值为 'hello'，描述为 '要发布的话题名称'
    # topic_arg = DeclareLaunchArgument(...)
    topic_arg = None

    # TODO 2: 用 LaunchConfiguration 获取 'topic_name' 的值
    # topic_config = LaunchConfiguration(...)
    topic_config = None

    # TODO 3: 把 topic_config 通过 parameters 传给节点
    #         格式: parameters=[{'topic_name': topic_config}]
    node = Node(
        package='ros2lings_30_launch_arguments',
        executable='configurable_node.py',
        name='configurable_node',
        output='screen',
        # TODO 3: 在这里添加 parameters 参数
    )

    # 注意: topic_arg 也需要加入 LaunchDescription
    return LaunchDescription([node])
