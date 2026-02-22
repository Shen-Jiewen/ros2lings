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
        os.path.dirname(__file__), '..', 'src', 'tf_listener_py.py'
    )
    spec = importlib.util.spec_from_file_location('tf_listener_py', src_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_node_can_be_created():
    """测试节点能否正常创建"""
    module = _load_module()
    node = module.TfListenerNode()
    assert node is not None
    node.destroy_node()


def test_buffer_exists():
    """测试 Buffer 存在且类型正确"""
    from tf2_ros import Buffer
    module = _load_module()
    node = module.TfListenerNode()
    assert hasattr(node, 'buffer_'), '节点应有 buffer_ 属性'
    assert isinstance(node.buffer_, Buffer), \
        f'buffer_ 应为 tf2_ros.Buffer 类型，实际为 {type(node.buffer_)}'
    node.destroy_node()


def test_listener_exists():
    """测试 TransformListener 存在且类型正确"""
    from tf2_ros import TransformListener
    module = _load_module()
    node = module.TfListenerNode()
    assert hasattr(node, 'listener_'), '节点应有 listener_ 属性'
    assert isinstance(node.listener_, TransformListener), \
        f'listener_ 应为 tf2_ros.TransformListener 类型，实际为 {type(node.listener_)}'
    node.destroy_node()


def test_lookup_transform_frame_order():
    """测试 lookup_transform 的帧参数顺序正确"""
    import inspect
    module = _load_module()
    source_code = inspect.getsource(module.TfListenerNode.timer_callback)

    # 检查 lookup_transform 调用中 target_frame 在前、source_frame 在后
    # 正确顺序: lookup_transform('world', 'child_frame', ...)
    # 即 target_frame_ 在前
    assert 'self.target_frame_' in source_code, \
        'timer_callback 中应使用 self.target_frame_'
    assert 'self.source_frame_' in source_code, \
        'timer_callback 中应使用 self.source_frame_'

    # 验证 target 在 source 之前出现（即 target 是第一个参数）
    target_pos = source_code.index('self.target_frame_')
    source_pos = source_code.index('self.source_frame_')
    assert target_pos < source_pos, \
        'lookup_transform 的第一个参数应为 target_frame（world），' \
        '第二个参数应为 source_frame（child_frame）'


def test_exception_handling_type():
    """测试异常处理捕获的是 tf2 相关异常，而不是 ValueError"""
    import inspect
    module = _load_module()
    source_code = inspect.getsource(module.TfListenerNode.timer_callback)

    # 不应该用 except ValueError 来捕获 TF2 异常
    assert 'except ValueError' not in source_code, \
        '应捕获 tf2_ros.TransformException（或其子类），而不是 ValueError'

    # 应该捕获 tf2 相关的异常
    has_tf2_exception = (
        'except tf2_ros.TransformException' in source_code
        or 'except tf2_ros.LookupException' in source_code
        or 'TransformException' in source_code
    )
    assert has_tf2_exception, \
        '应捕获 tf2_ros.TransformException（或 tf2_ros.LookupException 等 tf2 异常）'


def test_node_does_not_crash_on_callback():
    """测试节点调用回调不会崩溃（异常被正确捕获）"""
    module = _load_module()
    node = module.TfListenerNode()

    # 触发回调 — 由于没有 TF 数据，如果异常处理正确，不应崩溃
    try:
        node.timer_callback()
    except Exception as e:
        pytest.fail(f'timer_callback 不应抛出未捕获的异常，但抛出了: {type(e).__name__}: {e}')
    finally:
        node.destroy_node()
