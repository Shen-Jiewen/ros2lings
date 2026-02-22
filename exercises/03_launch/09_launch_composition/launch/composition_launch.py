#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: launch_composition
# 模块: 03 - Launch & Parameters
# 难度: ★★★☆☆
#
# 学习目标:
#   学会使用 ComposableNodeContainer 和 ComposableNode
#   在 Launch 文件中组合多个组件节点到同一进程中运行。
#
# 说明:
#   publisher_component 和 subscriber_component 已经编译为共享库。
#   你需要在 Launch 文件中创建一个 ComposableNodeContainer，
#   并将两个组件加载到容器中。
#
# 步骤:
#   1. 导入 ComposableNodeContainer 和 ComposableNode
#   2. 创建 ComposableNodeContainer，指定容器名称和命名空间
#   3. 添加两个 ComposableNode，使用正确的 plugin 名称和包名
#   4. 实现完成后，删除文件顶部的 "# I AM NOT DONE"

from launch import LaunchDescription
# TODO: 导入 ComposableNodeContainer 和 ComposableNode
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # TODO 1: 创建 ComposableNodeContainer
    # 提示:
    #   - name: 'composition_container'
    #   - namespace: ''
    #   - package: 'rclcpp_components'
    #   - executable: 'component_container'
    #   - composable_node_descriptions: [...]  ← 在这里放入 ComposableNode 列表
    #
    # TODO 2: 在 composable_node_descriptions 中添加两个 ComposableNode:
    #   ComposableNode(
    #       package='ros2lings_36_launch_composition',
    #       plugin='PublisherComponent',
    #       name='publisher_component',
    #   )
    #   ComposableNode(
    #       package='ros2lings_36_launch_composition',
    #       plugin='SubscriberComponent',
    #       name='subscriber_component',
    #   )

    container = None

    return LaunchDescription([])
