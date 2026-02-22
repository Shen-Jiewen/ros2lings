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


def test_has_base_link():
    """验证存在名为 base_link 的 link"""
    root = _load_urdf()
    links = root.findall('link')
    link_names = [l.attrib.get('name', '') for l in links]
    assert 'base_link' in link_names, \
        f'应有名为 base_link 的 link，当前 link 名称为: {link_names}'


def test_base_link_has_visual():
    """验证 base_link 包含 visual 元素"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    visual = link.find('visual')
    assert visual is not None, 'base_link 应有 visual 元素'


def test_visual_has_box_geometry():
    """验证 visual 中包含正确的 box geometry，且 size 有 3 个值"""
    root = _load_urdf()
    link = root.find(".//link[@name='base_link']")
    assert link is not None, '找不到名为 base_link 的 link'
    geometry = link.find('.//visual/geometry')
    assert geometry is not None, 'visual 中应有 geometry 元素'
    box = geometry.find('box')
    assert box is not None, 'geometry 应包含 box'
    size = box.attrib.get('size', '')
    parts = size.split()
    assert len(parts) == 3, f'box size 应有 3 个值，实际为 "{size}"'
    for i, p in enumerate(parts):
        val = float(p)
        assert val > 0, f'box size 的第 {i+1} 个值应为正数，实际为 {val}'
