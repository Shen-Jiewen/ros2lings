#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: multi_node_launch
# 模块: 03 - Launch & Parameters
# 难度: ★☆☆☆☆
#
# 学习目标:
#   学会在一个 Launch 文件中同时启动多个节点。
#
# 说明:
#   下面的 Launch 文件试图同时启动 talker 和 listener 两个节点，
#   但有 3 个错误需要你修复。
#
# 步骤:
#   1. 把 listener 节点加入 LaunchDescription 列表
#   2. 修复节点名称 — 两个节点不能同名
#   3. 给 talker 节点添加 output='screen'
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    talker = Node(
        package='ros2lings_29_multi_node_launch',
        executable='talker_node.py',
        name='talker',
                                                    # Bug 3: 缺少 output='screen'
    )

    listener = Node(
        package='ros2lings_29_multi_node_launch',
        executable='listener_node.py',
        name='talker',                              # Bug 2: 名称与 talker 重复
        output='screen',
    )

    return LaunchDescription([talker])              # Bug 1: 只有 talker，缺少 listener
