#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: conditional_launch
# 模块: 03 - Launch & Parameters
# 难度: ★★★☆☆
#
# 学习目标:
#   学会使用 IfCondition、UnlessCondition 和 DeclareLaunchArgument
#   实现条件启动节点。
#
# 说明:
#   根据 Launch 参数 'use_sim' 的值，决定启动仿真节点还是真实硬件节点：
#   - use_sim=true  → 启动 sim_node.py
#   - use_sim=false → 启动 real_node.py
#
# 步骤:
#   1. 用 DeclareLaunchArgument 声明 'use_sim' 参数（默认值 'false'）
#   2. 用 IfCondition 包裹 sim_node，当 use_sim=true 时启动
#   3. 用 UnlessCondition 包裹 real_node，当 use_sim=false 时启动
#   4. 实现完成后，删除文件顶部的 "# I AM NOT DONE"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# TODO: 导入 IfCondition 和 UnlessCondition
# from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # TODO 1: 声明 'use_sim' Launch 参数，默认值为 'false'
    # use_sim_arg = DeclareLaunchArgument(
    #     'use_sim',
    #     default_value='false',
    #     description='是否使用仿真模式'
    # )
    use_sim_arg = None

    # TODO 2: 创建 sim_node，使用 IfCondition 当 use_sim=true 时启动
    # 提示: condition=IfCondition(LaunchConfiguration('use_sim'))
    sim_node = Node(
        package='ros2lings_34_conditional_launch',
        executable='sim_node.py',
        name='sim_node',
        output='screen',
        # TODO 2: 在这里添加 condition 参数
    )

    # TODO 3: 创建 real_node，使用 UnlessCondition 当 use_sim=false 时启动
    # 提示: condition=UnlessCondition(LaunchConfiguration('use_sim'))
    real_node = Node(
        package='ros2lings_34_conditional_launch',
        executable='real_node.py',
        name='real_node',
        output='screen',
        # TODO 3: 在这里添加 condition 参数
    )

    return LaunchDescription([])
