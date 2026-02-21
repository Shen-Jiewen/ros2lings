#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: subscriber_py
# 模块: 01 - Nodes & Topics
# 难度: ★☆☆☆☆
#
# 学习目标:
#   理解 rclpy 中 Subscription 的创建和回调处理机制。
#
# 说明:
#   下面的代码尝试创建一个订阅者节点，从话题接收字符串消息。
#   但代码中有几个错误需要你修复。
#
# 步骤:
#   1. 修复回调函数的参数签名——缺少 self 参数
#   2. 修复 create_subscription 的调用——缺少 QoS 参数
#   3. 修复话题名称——应该与发布者一致
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberPy(Node):

    def __init__(self):
        super().__init__('subscriber_py')

        # TODO: create_subscription 需要四个参数：消息类型、话题名、回调、QoS 深度
        # Bug 1: 话题名称错误，应该是 'chatter' 而不是 'wrong_topic'
        # Bug 2: 缺少 QoS 参数
        self.subscription_ = self.create_subscription(
            String,
            'wrong_topic',                # Bug: 话题名称应该是 'chatter'
            self.listener_callback        # Bug: 缺少第四个参数 QoS
        )

    # TODO: 回调函数签名错误——作为实例方法，第一个参数应该是 self
    # Bug: 缺少 self 参数
    def listener_callback(msg):
        print(f'Received: "{msg.data}"')


def main():
    rclpy.init()
    node = SubscriberPy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
