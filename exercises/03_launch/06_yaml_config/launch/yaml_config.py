#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: yaml_config
# 模块: 03 - Launch & Parameters
# 难度: ★★☆☆☆
#
# 学习目标:
#   学会在 Launch 文件中加载 YAML 配置文件，
#   将参数从文件传递给节点。
#
# 说明:
#   config/params.yaml 中定义了节点的参数。
#   你需要在 Launch 文件中正确加载这个 YAML 文件，
#   并把参数传递给 configurable_node.py 节点。
#
# 步骤:
#   1. 用 os.path.join 构建 YAML 文件的路径
#   2. 创建 Node action，通过 parameters=[yaml_path] 加载配置
#   3. 返回包含 node 的 LaunchDescription
#   4. 实现完成后，删除文件顶部的 "# I AM NOT DONE"

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO 1: 用 os.path.join 获取 YAML 配置文件的路径
    # 提示: 配置文件在本包的 config/params.yaml
    #       可以用 os.path.dirname(__file__) 获取当前文件的目录
    #       然后用 '..' 回到包根目录，再进入 config/
    # yaml_path = os.path.join(
    #     os.path.dirname(__file__), '..', 'config', 'params.yaml'
    # )
    yaml_path = None

    # TODO 2: 创建 Node action，用 parameters 加载 YAML 配置
    # 提示: parameters=[yaml_path]
    node = Node(
        package='ros2lings_33_yaml_config',
        executable='configurable_node.py',
        name='configurable_node',
        output='screen',
        # TODO 2: 在这里添加 parameters 参数
    )

    # TODO 3: 把 node 加入 LaunchDescription
    return LaunchDescription([])
