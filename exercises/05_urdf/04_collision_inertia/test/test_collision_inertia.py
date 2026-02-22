import pytest
import os
import xml.etree.ElementTree as ET


def _load_urdf():
    """加载并解析 URDF 文件"""
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf')
    tree = ET.parse(urdf_path)
    return tree.getroot()


def test_robot_has_name():
    """验证 <robot> 标签具有非空的 name 属性"""
    root = _load_urdf()
    assert root.tag == 'robot', f'根元素应为 robot，实际为 "{root.tag}"'
    assert 'name' in root.attrib and root.attrib['name'] != '', \
        'robot 标签必须有 name 属性'


def test_base_link_has_collision():
    """验证 base_link 有 collision 元素"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    collision = link.find('collision')
    assert collision is not None, \
        'base_link 缺少 <collision> 元素，物理仿真需要碰撞几何体'


def test_base_link_collision_has_geometry():
    """验证 base_link 的 collision 中有 geometry"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    collision = link.find('collision')
    assert collision is not None, 'base_link 缺少 collision 元素'
    geometry = collision.find('geometry')
    assert geometry is not None, \
        'base_link 的 collision 中应有 geometry 元素'
    # 检查 geometry 内有具体的形状元素
    shapes = list(geometry)
    assert len(shapes) > 0, \
        'collision 的 geometry 中应包含一个形状元素（box、cylinder、sphere 等）'


def test_base_link_has_inertial():
    """验证 base_link 有 inertial 元素"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    inertial = link.find('inertial')
    assert inertial is not None, \
        'base_link 缺少 <inertial> 元素，物理仿真需要惯性属性'


def test_base_link_inertial_has_mass():
    """验证 base_link 的 inertial 中有 mass 元素且 value > 0"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    inertial = link.find('inertial')
    assert inertial is not None, 'base_link 缺少 inertial 元素'
    mass = inertial.find('mass')
    assert mass is not None, \
        'inertial 中缺少 <mass> 元素'
    value = mass.attrib.get('value', '')
    assert value != '', 'mass 元素缺少 value 属性'
    assert float(value) > 0, \
        f'mass 的 value 应为正数，实际为 {value}'


def test_base_link_inertial_has_inertia():
    """验证 base_link 的 inertial 中有 inertia 元素且主对角线值 > 0"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    inertial = link.find('inertial')
    assert inertial is not None, 'base_link 缺少 inertial 元素'
    inertia = inertial.find('inertia')
    assert inertia is not None, \
        'inertial 中缺少 <inertia> 元素（惯性矩阵）'
    for attr in ['ixx', 'iyy', 'izz']:
        val = inertia.attrib.get(attr, '')
        assert val != '', f'inertia 缺少 {attr} 属性'
        assert float(val) > 0, \
            f'inertia 的 {attr} 应为正数，实际为 {val}'


def test_wheel_has_collision():
    """验证 wheel 有 collision 元素"""
    root = _load_urdf()
    link = root.find(".//link[@name='wheel']")
    assert link is not None, '找不到名为 wheel 的 link'
    collision = link.find('collision')
    assert collision is not None, \
        'wheel 缺少 <collision> 元素，物理仿真需要碰撞几何体'
    geometry = collision.find('geometry')
    assert geometry is not None, \
        'wheel 的 collision 中应有 geometry 元素'


def test_wheel_has_inertial():
    """验证 wheel 有 inertial 元素"""
    root = _load_urdf()
    link = root.find(".//link[@name='wheel']")
    assert link is not None, '找不到名为 wheel 的 link'
    inertial = link.find('inertial')
    assert inertial is not None, \
        'wheel 缺少 <inertial> 元素，物理仿真需要惯性属性'
    mass = inertial.find('mass')
    assert mass is not None, 'wheel 的 inertial 中缺少 mass 元素'
    assert float(mass.attrib.get('value', '0')) > 0, \
        'wheel 的 mass value 应为正数'
    inertia = inertial.find('inertia')
    assert inertia is not None, 'wheel 的 inertial 中缺少 inertia 元素'
