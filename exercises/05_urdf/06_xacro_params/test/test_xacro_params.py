import pytest
import os
import re


def _load_xacro_content():
    """加载 Xacro 文件的原始文本内容"""
    path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf.xacro')
    with open(path, 'r') as f:
        return f.read()


def test_has_arm_length_property():
    """应定义 xacro:property "arm_length" """
    content = _load_xacro_content()
    pattern = r'xacro:property\s+name="arm_length"\s+value="[^"]*"'
    assert re.search(pattern, content), \
        '应定义 xacro:property name="arm_length"，例如: ' \
        '<xacro:property name="arm_length" value="0.5" />'


def test_has_arm_radius_property():
    """应定义 xacro:property "arm_radius" """
    content = _load_xacro_content()
    pattern = r'xacro:property\s+name="arm_radius"\s+value="[^"]*"'
    assert re.search(pattern, content), \
        '应定义 xacro:property name="arm_radius"，例如: ' \
        '<xacro:property name="arm_radius" value="0.05" />'


def test_has_arm_segment_macro():
    """应定义 xacro:macro "arm_segment" """
    content = _load_xacro_content()
    pattern = r'xacro:macro\s+name="arm_segment"'
    assert re.search(pattern, content), \
        '应定义 xacro:macro name="arm_segment"'


def test_macro_has_required_params():
    """arm_segment 宏应包含 name, parent, length, radius 参数"""
    content = _load_xacro_content()
    # 找到 macro 定义的 params 属性
    match = re.search(r'xacro:macro\s+name="arm_segment"\s+params="([^"]*)"', content)
    assert match, '应定义 xacro:macro name="arm_segment" 且带有 params 属性'
    params_str = match.group(1)
    for param in ['name', 'parent', 'length', 'radius']:
        assert param in params_str, \
            f'arm_segment 宏的 params 中应包含 "{param}"，当前 params="{params_str}"'


def test_macro_called_twice():
    """arm_segment 宏应被调用至少 2 次"""
    content = _load_xacro_content()
    calls = re.findall(r'<xacro:arm_segment\s', content)
    assert len(calls) >= 2, \
        f'arm_segment 宏应被调用至少 2 次，实际调用了 {len(calls)} 次'


def test_has_base_link():
    """应有 base_link 定义"""
    content = _load_xacro_content()
    assert re.search(r'<link\s+name="base_link"', content), \
        '应有 <link name="base_link"> 定义'


def test_has_revolute_joint():
    """宏中应包含 revolute 类型的 joint"""
    content = _load_xacro_content()
    assert re.search(r'type="revolute"', content), \
        '应有 type="revolute" 的 joint（在宏定义或调用中）'


def test_has_cylinder_geometry():
    """宏中应使用 cylinder 几何体"""
    content = _load_xacro_content()
    assert '<cylinder' in content, \
        '臂段应使用 <cylinder> 几何体'


def test_macro_has_joint_limits():
    """revolute joint 应有 axis 和 limit 元素"""
    content = _load_xacro_content()
    assert '<axis' in content, \
        'revolute joint 应有 <axis> 元素指定旋转轴'
    assert '<limit' in content, \
        'revolute joint 应有 <limit> 元素指定运动范围'
