#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: hello_node_py
# 模块: 01 - Nodes & Topics
# 难度: ★☆☆☆☆
#
# 学习目标:
#   理解 rclpy 节点的创建和 rclpy.init/shutdown 生命周期。
#
# 说明:
#   下面的代码尝试创建一个最简单的 Python ROS2 节点并让它打印一条消息。
#   但代码中有几个错误需要你修复。
#
# 步骤:
#   1. 修复 rclpy.init() 的调用顺序——必须在创建节点之前初始化
#   2. 修复节点的创建方式
#   3. 确保节点能正确 spin 一次
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import rclpy
from rclpy.node import Node


def main():
    # TODO: rclpy.init() 必须在创建节点之前调用
    # 提示: 把 rclpy.init() 移到正确的位置
    node = Node('hello_node_py')  # Bug: 在 init 之前创建节点

    rclpy.init()  # Bug: init 应该在节点创建之前

    node.get_logger().info('Hello from Python ROS2 node!')

    # TODO: 让节点至少 spin 一次
    # 提示: 使用 rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
