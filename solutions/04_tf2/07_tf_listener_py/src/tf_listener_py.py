#!/usr/bin/env python3
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

        # 正确：TransformListener 传入 buffer 和 node
        self.listener_ = TransformListener(self.buffer_, self)

        # 定时器，每 500ms 查询一次变换
        self.timer_ = self.create_timer(0.5, self.timer_callback)

        self.target_frame_ = 'world'
        self.source_frame_ = 'child_frame'

        self.get_logger().info('TF 监听器（Python）已启动，等待变换数据...')

    def timer_callback(self):
        # 正确：捕获 tf2_ros.TransformException
        try:
            # 正确：lookup_transform(target_frame, source_frame, time)
            t = self.buffer_.lookup_transform(
                self.target_frame_,   # 目标帧: 'world'
                self.source_frame_,   # 源帧: 'child_frame'
                rclpy.time.Time())

            self.get_logger().info(
                f'变换 {self.target_frame_} -> {self.source_frame_}: '
                f'[{t.transform.translation.x:.2f}, '
                f'{t.transform.translation.y:.2f}, '
                f'{t.transform.translation.z:.2f}]')

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'无法获取变换: {ex}')


def main():
    rclpy.init()
    node = TfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
