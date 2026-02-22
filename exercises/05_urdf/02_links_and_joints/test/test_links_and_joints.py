import pytest
import os
import xml.etree.ElementTree as ET


# 有效的 URDF joint 类型
VALID_JOINT_TYPES = {'revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar'}


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


def test_has_expected_link_names():
    """验证 link 名称为 base_link, upper_arm, lower_arm"""
    root = _load_urdf()
    links = root.findall('link')
    link_names = {l.attrib.get('name', '') for l in links}
    expected = {'base_link', 'upper_arm', 'lower_arm'}
    assert link_names == expected, \
        f'link 名称应为 {expected}，实际为 {link_names}'


def test_has_two_joints():
    """验证机器人有 2 个 joint"""
    root = _load_urdf()
    joints = root.findall('joint')
    assert len(joints) == 2, \
        f'应有 2 个 joint，实际有 {len(joints)} 个'


def test_joint_types_are_valid():
    """验证所有 joint 的 type 属性都是有效值"""
    root = _load_urdf()
    joints = root.findall('joint')
    for joint in joints:
        jname = joint.attrib.get('name', '(未命名)')
        jtype = joint.attrib.get('type', '')
        assert jtype in VALID_JOINT_TYPES, \
            f'joint "{jname}" 的 type 为 "{jtype}"，不是有效的关节类型。' \
            f'有效类型为: {VALID_JOINT_TYPES}'


def test_joint_parent_child_exist():
    """验证 joint 的 parent 和 child link 名称在已定义的 link 中"""
    root = _load_urdf()
    links = root.findall('link')
    link_names = {l.attrib.get('name', '') for l in links}

    joints = root.findall('joint')
    for joint in joints:
        jname = joint.attrib.get('name', '(未命名)')

        parent = joint.find('parent')
        assert parent is not None, f'joint "{jname}" 缺少 <parent> 元素'
        parent_link = parent.attrib.get('link', '')
        assert parent_link in link_names, \
            f'joint "{jname}" 的 parent link "{parent_link}" ' \
            f'不在已定义的 link 列表中: {link_names}'

        child = joint.find('child')
        assert child is not None, f'joint "{jname}" 缺少 <child> 元素'
        child_link = child.attrib.get('link', '')
        assert child_link in link_names, \
            f'joint "{jname}" 的 child link "{child_link}" ' \
            f'不在已定义的 link 列表中: {link_names}'


def test_revolute_joint_has_axis_and_limit():
    """验证 revolute 类型的 joint 有 <axis> 和 <limit> 元素"""
    root = _load_urdf()
    joints = root.findall('joint')
    for joint in joints:
        jname = joint.attrib.get('name', '(未命名)')
        jtype = joint.attrib.get('type', '')
        if jtype == 'revolute':
            axis = joint.find('axis')
            assert axis is not None, \
                f'revolute joint "{jname}" 缺少 <axis> 元素，' \
                '必须用 <axis xyz="..."/> 指定旋转轴'

            limit = joint.find('limit')
            assert limit is not None, \
                f'revolute joint "{jname}" 缺少 <limit> 元素，' \
                '必须用 <limit lower="..." upper="..." effort="..." velocity="..."/> 指定运动范围'

            # 检查 limit 的必需属性
            assert 'lower' in limit.attrib, \
                f'revolute joint "{jname}" 的 <limit> 缺少 lower 属性'
            assert 'upper' in limit.attrib, \
                f'revolute joint "{jname}" 的 <limit> 缺少 upper 属性'
            assert 'effort' in limit.attrib, \
                f'revolute joint "{jname}" 的 <limit> 缺少 effort 属性'
            assert 'velocity' in limit.attrib, \
                f'revolute joint "{jname}" 的 <limit> 缺少 velocity 属性'
