#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: tf_listener_py
# 模块: 04 - TF2 Transforms
# 难度: ★★☆☆☆
#
# 学习目标:
#   掌握 Python 中 tf2_ros.Buffer 和 tf2_ros.TransformListener 的配合使用，
#   理解 lookup_transform 的参数含义和异常处理方式。
#
# 说明:
#   下面的节点试图查询从 "world" 到 "child_frame" 的坐标变换，
#   但有 3 个错误需要你修复。
#
# 步骤:
#   1. 修复 TransformListener 的创建 — 必须传入 buffer
#   2. 修复 lookup_transform 的帧参数顺序 — source 和 target 反了
#   3. 修复异常处理 — 应该捕获 tf2_ros 的 TransformException 而非 ValueError
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TfListenerNode(Node):

    def __init__(self):
        super().__init__('tf_listener_py_node')

        # 创建 Buffer
        self.buffer_ = Buffer(node=self)

        # BUG 1: TransformListener 创建时没有传入 buffer
        # 正确做法: self.listener_ = TransformListener(self.buffer_, self)
        self.listener_ = TransformListener(node=self)

        # 定时器，每 500ms 查询一次变换
        self.timer_ = self.create_timer(0.5, self.timer_callback)

        self.target_frame_ = 'world'
        self.source_frame_ = 'child_frame'

        self.get_logger().info('TF 监听器（Python）已启动，等待变换数据...')

    def timer_callback(self):
        # BUG 3: 异常处理类型错误 — 捕获了错误的异常类型
        try:
            # BUG 2: lookup_transform 的参数顺序是 (target_frame, source_frame, time)
            # 这里写反了：source 和 target 的位置对调了
            t = self.buffer_.lookup_transform(
                self.source_frame_,  # 应该是 target_frame: 'world'
                self.target_frame_,  # 应该是 source_frame: 'child_frame'
                rclpy.time.Time())

            self.get_logger().info(
                f'变换 {self.target_frame_} -> {self.source_frame_}: '
                f'[{t.transform.translation.x:.2f}, '
                f'{t.transform.translation.y:.2f}, '
                f'{t.transform.translation.z:.2f}]')

        except ValueError as ex:
            self.get_logger().warn(f'无法获取变换: {ex}')


def main():
    rclpy.init()
    node = TfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
