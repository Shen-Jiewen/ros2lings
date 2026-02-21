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


def test_can_create_publisher():
    """测试能否成功创建发布者"""
    node = Node('test_pub_create')
    pub = node.create_publisher(String, 'chatter', 10)
    assert pub is not None
    node.destroy_node()


def test_publisher_publishes_message():
    """测试发布者能否成功发布消息"""
    node = Node('test_pub_msg')
    pub = node.create_publisher(String, 'test_chatter', 10)

    msg = String()
    msg.data = 'Hello from test'
    pub.publish(msg)

    # 验证发布者的话题名称
    topic_names = [t[0] for t in node.get_topic_names_and_types()]
    assert '/test_chatter' in topic_names
    node.destroy_node()


def test_timer_callback_bound():
    """测试定时器能否正确绑定回调函数"""
    callback_called = [False]

    class TestNode(Node):
        def __init__(self):
            super().__init__('test_timer_node')
            self.timer = self.create_timer(0.1, self.cb)

        def cb(self):
            callback_called[0] = True

    node = TestNode()
    rclpy.spin_once(node, timeout_sec=0.5)
    assert callback_called[0], '定时器回调未被调用'
    node.destroy_node()
