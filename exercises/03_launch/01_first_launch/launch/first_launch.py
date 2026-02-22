#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: first_launch
# 模块: 03 - Launch & Parameters
# 难度: ★☆☆☆☆
#
# 学习目标:
#   理解 Launch 文件的基本结构，学会用 Launch 文件启动一个节点。
#
# 说明:
#   下面的 Launch 文件试图启动 first_launch_node.py 节点，
#   但有 3 个错误需要你修复。
#
# 步骤:
#   1. 修复 package 名称 — 必须与 package.xml 中的名称一致
#   2. 修复 executable 名称 — 必须与 CMakeLists.txt 中安装的文件名一致
#   3. 把 node 加入 LaunchDescription 列表中
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='wrong_package_name',       # Bug 1: 包名不对
        executable='wrong_node',            # Bug 2: 可执行文件名不对
        name='first_launch_node',
        output='screen',
    )

    return LaunchDescription([])            # Bug 3: 列表为空，没有启动任何节点
