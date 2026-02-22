#!/usr/bin/env python3
import pytest
import importlib.util
import os


def _load_quiz_module():
    """加载问答模块"""
    src_file = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'tf_debug_quiz.py'
    )
    spec = importlib.util.spec_from_file_location('tf_debug_quiz', src_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_view_frames_tool():
    """测试: 生成 TF 树 PDF 图的命令行工具"""
    module = _load_quiz_module()
    assert module.answer_view_frames_tool == 'view_frames', \
        'view_frames 是 tf2_tools 包中用于生成 TF 树 PDF 图的工具'


def test_static_topic():
    """测试: 静态变换发布的话题"""
    module = _load_quiz_module()
    assert module.answer_static_topic == '/tf_static', \
        '静态变换发布在 /tf_static 话题上'


def test_dynamic_topic():
    """测试: 动态变换发布的话题"""
    module = _load_quiz_module()
    assert module.answer_dynamic_topic == '/tf', \
        '动态变换发布在 /tf 话题上'


def test_transform_direction():
    """测试: lookupTransform 返回的变换方向"""
    module = _load_quiz_module()
    assert module.answer_transform_direction == 'source_to_target', \
        'lookupTransform(target, source, time) 返回从 source 到 target 的变换'


def test_multi_parent():
    """测试: TF 树中多父节点的处理"""
    module = _load_quiz_module()
    assert module.answer_multi_parent == 'error', \
        'TF2 不允许一个子帧有多个父帧，会产生错误'
