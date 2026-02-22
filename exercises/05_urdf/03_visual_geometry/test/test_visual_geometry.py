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


def test_has_three_links():
    """验证机器人有 3 个 link"""
    root = _load_urdf()
    links = root.findall('link')
    assert len(links) == 3, \
        f'应有 3 个 link，实际有 {len(links)} 个'


def test_has_two_joints():
    """验证机器人有 2 个 joint"""
    root = _load_urdf()
    joints = root.findall('joint')
    assert len(joints) == 2, \
        f'应有 2 个 joint，实际有 {len(joints)} 个'


def test_base_link_has_box_geometry():
    """验证 base_link 的 visual 中包含 box 几何体"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    geometry = link.find('.//visual/geometry')
    assert geometry is not None, 'base_link 的 visual 中应有 geometry 元素'
    box = geometry.find('box')
    assert box is not None, \
        'base_link 的 geometry 应包含 box 元素'
    size = box.attrib.get('size', '')
    parts = size.split()
    assert len(parts) == 3, \
        f'box 的 size 应有 3 个值，实际为 "{size}"'


def test_body_has_cylinder_geometry():
    """验证 body 的 visual 中包含 cylinder 几何体"""
    root = _load_urdf()
    link = root.find(".//link[@name='body']")
    assert link is not None, '找不到名为 body 的 link'
    geometry = link.find('.//visual/geometry')
    assert geometry is not None, 'body 的 visual 中应有 geometry 元素'
    cylinder = geometry.find('cylinder')
    assert cylinder is not None, \
        'body 的 geometry 应包含 cylinder 元素'
    assert 'radius' in cylinder.attrib, \
        'cylinder 缺少 radius 属性'
    assert 'length' in cylinder.attrib, \
        'cylinder 缺少 length 属性'
    assert float(cylinder.attrib['radius']) > 0, \
        'cylinder 的 radius 应为正数'
    assert float(cylinder.attrib['length']) > 0, \
        'cylinder 的 length 应为正数'


def test_head_has_sphere_geometry():
    """验证 head 的 visual 中包含 sphere 几何体"""
    root = _load_urdf()
    link = root.find(".//link[@name='head']")
    assert link is not None, '找不到名为 head 的 link'
    geometry = link.find('.//visual/geometry')
    assert geometry is not None, 'head 的 visual 中应有 geometry 元素'
    sphere = geometry.find('sphere')
    assert sphere is not None, \
        'head 的 geometry 应包含 sphere 元素'
    assert 'radius' in sphere.attrib, \
        'sphere 缺少 radius 属性'
    assert float(sphere.attrib['radius']) > 0, \
        'sphere 的 radius 应为正数'


def test_each_visual_has_material():
    """验证每个 link 的 visual 元素中都有 material"""
    root = _load_urdf()
    link_names = ['base_link', 'body', 'head']
    for name in link_names:
        link = root.find(f".//link[@name='{name}']")
        assert link is not None, f'找不到名为 {name} 的 link'
        visual = link.find('visual')
        assert visual is not None, f'{name} 应有 visual 元素'
        material = visual.find('material')
        assert material is not None, \
            f'{name} 的 visual 中应有 material 元素'
        assert 'name' in material.attrib, \
            f'{name} 的 material 应有 name 属性'
        color = material.find('color')
        assert color is not None, \
            f'{name} 的 material 中应有 color 元素'
        assert 'rgba' in color.attrib, \
            f'{name} 的 color 应有 rgba 属性'
