#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_can_create_subscription():
    """测试能否成功创建订阅者"""
    node = Node('test_sub_create')
    received = []

    sub = node.create_subscription(
        String,
        'chatter',
        lambda msg: received.append(msg),
        10
    )
    assert sub is not None
    node.destroy_node()


def test_subscription_receives_message():
    """测试订阅者能否接收到消息"""
    node = Node('test_sub_recv')
    received = []

    node.create_subscription(
        String,
        'test_sub_topic',
        lambda msg: received.append(msg.data),
        10
    )
    pub = node.create_publisher(String, 'test_sub_topic', 10)

    msg = String()
    msg.data = 'Hello from test'
    pub.publish(msg)

    # spin 让消息被处理
    rclpy.spin_once(node, timeout_sec=0.5)

    assert len(received) > 0, '订阅者未收到任何消息'
    assert received[0] == 'Hello from test'
    node.destroy_node()


def test_callback_has_correct_signature():
    """测试回调函数签名正确——实例方法需要 self 和 msg 参数"""

    class TestSubNode(Node):
        def __init__(self):
            super().__init__('test_cb_sig')
            self.received_data = None
            self.create_subscription(
                String,
                'test_cb_topic',
                self.callback,
                10
            )
            self.pub = self.create_publisher(String, 'test_cb_topic', 10)

        def callback(self, msg):
            self.received_data = msg.data

    node = TestSubNode()
    msg = String()
    msg.data = 'test signature'
    node.pub.publish(msg)

    rclpy.spin_once(node, timeout_sec=0.5)

    assert node.received_data == 'test signature', '回调函数签名错误或未被调用'
    node.destroy_node()
