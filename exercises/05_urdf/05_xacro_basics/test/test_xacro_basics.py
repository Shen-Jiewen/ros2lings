import pytest
import os
import re


def _load_xacro_content():
    """加载 Xacro 文件的原始文本内容"""
    path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf.xacro')
    with open(path, 'r') as f:
        return f.read()


def test_xacro_namespace_correct():
    """命名空间 URL 必须正确: http://www.ros.org/wiki/xacro"""
    content = _load_xacro_content()
    # 检查 xmlns:xacro 属性中的 URL，而不只是文件中任意位置
    assert re.search(r'xmlns:xacro\s*=\s*"http://www\.ros\.org/wiki/xacro"', content), \
        '命名空间应为 xmlns:xacro="http://www.ros.org/wiki/xacro"（注意末尾是 xacro 不是 xacr）'


def test_property_reference_syntax():
    """属性引用应使用 ${} 语法，而非 $()"""
    content = _load_xacro_content()
    # 检查 box size 行中不应有 $() 语法
    box_lines = [l for l in content.split('\n') if 'box size' in l.lower()]
    for line in box_lines:
        if '$' in line:
            assert '${' in line, \
                f'属性引用语法错误: {line.strip()}。应使用 ${{}} 而非 $()'
            assert '$(' not in line, \
                f'属性引用语法错误: {line.strip()}。应使用 ${{}} 而非 $()'


def test_macro_parameter_names():
    """宏调用的参数名必须与宏定义中的参数名一致"""
    content = _load_xacro_content()
    # 不应有 widht（width 的常见拼写错误）
    assert 'widht' not in content, \
        '参数名拼写错误: "widht" 应为 "width"'


def test_has_two_links():
    """Xacro 文件应通过宏调用定义至少两个 link"""
    content = _load_xacro_content()
    # 匹配宏调用: xacro:box_link name="..."
    calls = re.findall(r'xacro:box_link\s+name="(\w+)"', content)
    assert len(calls) >= 2, \
        f'应有至少 2 个 box_link 宏调用，实际有 {len(calls)} 个'


def test_has_joint():
    """应有一个 joint 连接两个 link"""
    content = _load_xacro_content()
    assert '<joint' in content, '应有 <joint> 元素连接 link'


def test_has_xacro_properties():
    """应使用 xacro:property 定义属性"""
    content = _load_xacro_content()
    props = re.findall(r'xacro:property\s+name="(\w+)"', content)
    assert 'base_width' in props, '应定义 xacro:property "base_width"'
    assert 'base_length' in props, '应定义 xacro:property "base_length"'
    assert 'base_height' in props, '应定义 xacro:property "base_height"'


def test_has_xacro_macro_definition():
    """应使用 xacro:macro 定义宏"""
    content = _load_xacro_content()
    macros = re.findall(r'xacro:macro\s+name="(\w+)"', content)
    assert 'box_link' in macros, '应定义 xacro:macro "box_link"'
