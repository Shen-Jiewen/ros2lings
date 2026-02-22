#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: service_py
# 模块: 02 - Services & Actions
# 难度: ★☆☆☆☆
#
# 学习目标:
#   理解 Python 中 ROS2 服务的创建方式，包括导入路径、
#   回调函数签名和服务名称的一致性。
#
# 说明:
#   下面的代码尝试创建一个 Python AddTwoInts 服务服务器和客户端，
#   但代码中有三个错误。你需要找到并修复它们。
#
# 步骤:
#   1. 修复导入路径（包名拼写错误）
#   2. 修复回调函数签名（缺少 self 参数）
#   3. 修复服务名称不一致的问题
#   4. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

import rclpy
from rclpy.node import Node
from example_interface.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_numbers', self.handle_add)
        self.get_logger().info('Service server ready.')

    def handle_add(request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main():
    rclpy.init()
    server = AddTwoIntsServer()

    # 创建客户端节点
    client_node = Node('add_two_ints_client')
    client = client_node.create_client(AddTwoInts, 'add_two_ints')

    # 等待服务可用
    client.wait_for_service(timeout_sec=5.0)

    # 发送请求
    request = AddTwoInts.Request()
    request.a = 10
    request.b = 20

    future = client.call_async(request)

    # TODO: 使用 executor 同时 spin 两个节点
    # 提示: SingleThreadedExecutor, add_node, spin_until_future_complete
    rclpy.spin_until_future_complete(server, future)

    result = future.result()
    client_node.get_logger().info(f'Result: {result.sum}')

    server.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
