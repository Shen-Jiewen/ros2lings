#!/usr/bin/env python3
import pytest
import importlib.util
import os


def _load_quiz_module():
    """加载问答模块"""
    src_file = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'urdf_tf_quiz.py'
    )
    spec = importlib.util.spec_from_file_location('urdf_tf_quiz', src_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_urdf_parameter():
    """测试: robot_state_publisher 读取 URDF 的参数名"""
    module = _load_quiz_module()
    assert module.answer_urdf_parameter == 'robot_description', \
        'robot_state_publisher 通过 robot_description 参数读取 URDF'


def test_fixed_joint_topic():
    """测试: fixed joint 变换发布的话题"""
    module = _load_quiz_module()
    assert module.answer_fixed_joint_topic == '/tf_static', \
        'robot_state_publisher 将 fixed joint 的变换发布到 /tf_static'


def test_movable_joint_publisher():
    """测试: 发布可动关节状态的节点"""
    module = _load_quiz_module()
    assert module.answer_movable_joint_publisher == 'joint_state_publisher', \
        'joint_state_publisher 负责发布 revolute/prismatic 等可动关节的状态'


def test_joint_states_topic():
    """测试: joint_state_publisher 发布的话题"""
    module = _load_quiz_module()
    assert module.answer_joint_states_topic == '/joint_states', \
        'joint_state_publisher 将关节状态发布到 /joint_states 话题'


def test_tf_frame_count():
    """测试: URDF 产生的 TF 帧数"""
    module = _load_quiz_module()
    assert module.answer_tf_frame_count == 'N', \
        '一个有 N 个 link 的 URDF 会产生 N 个 TF 帧（每个 link 对应一个帧）'
