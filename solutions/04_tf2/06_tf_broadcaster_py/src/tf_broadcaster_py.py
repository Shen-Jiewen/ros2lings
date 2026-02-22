#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

# 正确：使用 TransformBroadcaster（动态广播器）
from tf2_ros import TransformBroadcaster


class TfBroadcasterNode(Node):

    def __init__(self):
        super().__init__('tf_broadcaster_node')

        # 正确使用 TransformBroadcaster
        self.broadcaster_ = TransformBroadcaster(self)

        # 创建定时器，每 100ms 广播一次变换
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('TF 动态变换广播器（Python）已启动')

    def timer_callback(self):
        t = TransformStamped()

        # 正确：设置当前时间戳
        t.header.stamp = self.get_clock().now().to_msg()

        # 正确：frame_id 是父帧，child_frame_id 是子帧
        t.header.frame_id = 'world'
        t.child_frame_id = 'child_frame'

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
