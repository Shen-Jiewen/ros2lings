#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: tf_broadcaster_py
# 模块: 04 - TF2 Transforms
# 难度: ★☆☆☆☆
#
# 学习目标:
#   掌握 Python 中 TransformBroadcaster 的使用，理解动态变换广播的基本模式，
#   包括广播器类型选择、时间戳设置和帧 ID 的正确配置。
#
# 说明:
#   下面的节点试图在定时器回调中持续广播一个动态坐标变换
#   （world -> child_frame），但有 3 个错误需要你修复。
#
# 步骤:
#   1. 修复导入 — 应该使用 TransformBroadcaster，而不是 StaticTransformBroadcaster
#   2. 设置 TransformStamped 的时间戳 — 使用 self.get_clock().now().to_msg()
#   3. 修复 frame_id 和 child_frame_id 的顺序 — 它们被设置反了
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

# BUG 1: 应该导入 TransformBroadcaster，而不是 StaticTransformBroadcaster
#         动态变换需要 TransformBroadcaster，静态变换才用 StaticTransformBroadcaster
from tf2_ros import StaticTransformBroadcaster


class TfBroadcasterNode(Node):

    def __init__(self):
        super().__init__('tf_broadcaster_node')

        # 创建广播器（这里使用了错误的类型，见 BUG 1）
        self.broadcaster_ = StaticTransformBroadcaster(self)

        # 创建定时器，每 100ms 广播一次变换
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('TF 动态变换广播器（Python）已启动')

    def timer_callback(self):
        t = TransformStamped()

        # BUG 2: 没有设置时间戳
        # 动态变换必须在每次回调中设置当前时间戳
        # 正确做法: t.header.stamp = self.get_clock().now().to_msg()

        # BUG 3: frame_id 和 child_frame_id 设置反了
        # frame_id 应该是父帧 "world"，child_frame_id 应该是子帧 "child_frame"
        t.header.frame_id = 'child_frame'
        t.child_frame_id = 'world'

        # 模拟随时间变化的圆周运动
        seconds = self.get_clock().now().nanoseconds / 1e9
        t.transform.translation.x = 2.0 * math.cos(seconds)
        t.transform.translation.y = 2.0 * math.sin(seconds)
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster_.sendTransform(t)


def main():
    rclpy.init()
    node = TfBroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
