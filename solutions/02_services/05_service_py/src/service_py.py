#!/usr/bin/env python3
# 参考答案 — service_py
# 修复：导入路径、self 参数、服务名称统一

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ros2lings_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.handle_add)
        self.get_logger().info('Service server ready.')

    def handle_add(self, request, response):
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
    if not client.wait_for_service(timeout_sec=5.0):
        client_node.get_logger().error('Service not available')
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
        return

    # 发送请求
    request = AddTwoInts.Request()
    request.a = 10
    request.b = 20

    future = client.call_async(request)

    # 使用 executor 同时 spin 两个节点
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    executor.spin_until_future_complete(future)

    result = future.result()
    client_node.get_logger().info(f'Result: {result.sum}')

    server.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
