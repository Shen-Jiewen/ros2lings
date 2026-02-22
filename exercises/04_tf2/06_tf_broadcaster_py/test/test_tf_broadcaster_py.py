#!/usr/bin/env python3
import pytest
import rclpy


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def _load_module():
    """动态加载源文件模块"""
    import importlib.util
    import os
    src_file = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'tf_broadcaster_py.py'
    )
    spec = importlib.util.spec_from_file_location('tf_broadcaster_py', src_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_node_can_be_created():
    """测试节点能否正常创建"""
    module = _load_module()
    node = module.TfBroadcasterNode()
    assert node is not None
    node.destroy_node()


def test_broadcaster_is_dynamic():
    """测试使用的是 TransformBroadcaster（动态），而不是 StaticTransformBroadcaster"""
    from tf2_ros import TransformBroadcaster
    module = _load_module()
    node = module.TfBroadcasterNode()
    assert isinstance(node.broadcaster_, TransformBroadcaster), \
        '应该使用 TransformBroadcaster，而不是 StaticTransformBroadcaster'
    node.destroy_node()


def test_frame_id_is_world():
    """测试 frame_id（父帧）设置为 world"""
    from geometry_msgs.msg import TransformStamped
    module = _load_module()
    node = module.TfBroadcasterNode()

    # 手动调用回调，检查广播的变换帧 ID
    # 通过 monkey-patch sendTransform 来捕获发送的消息
    captured = []
    original_send = node.broadcaster_.sendTransform

    def capture_send(transform):
        captured.append(transform)
        return original_send(transform)

    node.broadcaster_.sendTransform = capture_send
    node.timer_callback()

    assert len(captured) > 0, '回调应调用 sendTransform'
    assert captured[0].header.frame_id == 'world', \
        f'frame_id 应为 "world"，实际为 "{captured[0].header.frame_id}"'
    node.destroy_node()


def test_child_frame_id_is_child_frame():
    """测试 child_frame_id（子帧）设置为 child_frame"""
    module = _load_module()
    node = module.TfBroadcasterNode()

    captured = []
    original_send = node.broadcaster_.sendTransform

    def capture_send(transform):
        captured.append(transform)
        return original_send(transform)

    node.broadcaster_.sendTransform = capture_send
    node.timer_callback()

    assert len(captured) > 0, '回调应调用 sendTransform'
    assert captured[0].child_frame_id == 'child_frame', \
        f'child_frame_id 应为 "child_frame"，实际为 "{captured[0].child_frame_id}"'
    node.destroy_node()


def test_timestamp_is_set():
    """测试时间戳已设置（不为零）"""
    module = _load_module()
    node = module.TfBroadcasterNode()

    captured = []
    original_send = node.broadcaster_.sendTransform

    def capture_send(transform):
        captured.append(transform)
        return original_send(transform)

    node.broadcaster_.sendTransform = capture_send
    node.timer_callback()

    assert len(captured) > 0, '回调应调用 sendTransform'
    stamp = captured[0].header.stamp
    assert not (stamp.sec == 0 and stamp.nanosec == 0), \
        '时间戳不应为零，应使用 self.get_clock().now().to_msg() 设置'
    node.destroy_node()
