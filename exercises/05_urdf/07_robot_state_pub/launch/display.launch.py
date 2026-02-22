#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: robot_state_pub
# 模块: 05 - URDF & Robot Modeling
# 难度: ★★☆☆☆
#
# 学习目标:
#   学会使用 Launch 文件启动 robot_state_publisher，
#   将 URDF 模型加载到 ROS2 参数系统中。
#
# 说明:
#   你需要编写一个 Launch 文件，完成以下任务：
#   1. 读取 urdf/robot.urdf 文件内容
#   2. 启动 robot_state_publisher 节点，将 URDF 内容作为参数传入
#
# 步骤:
#   1. 用 os.path.join 构建 URDF 文件路径
#   2. 用 open() 读取 URDF 内容
#   3. 创建 robot_state_publisher 节点，传入 robot_description 参数
#   4. 返回包含该节点的 LaunchDescription
#   5. 实现完成后，删除文件顶部的 "# I AM NOT DONE"

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO 1: 构建 URDF 文件路径
    # 提示: 使用 os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf')
    urdf_path = None

    # TODO 2: 读取 URDF 文件内容
    # 提示: 使用 open(urdf_path, 'r') 读取文件，将内容赋值给 robot_description
    robot_description = None

    # TODO 3: 创建 robot_state_publisher 节点
    # 提示: 使用 Node() 创建节点，package 和 executable 都是 'robot_state_publisher'
    #       通过 parameters=[{'robot_description': robot_description}] 传入 URDF
    robot_state_pub_node = None

    # TODO 4: 返回包含该节点的 LaunchDescription
    return LaunchDescription([])
