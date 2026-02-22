#!/usr/bin/env python3
import pytest
import importlib.util
import os


def _load_quiz_module():
    """加载问答模块"""
    src_file = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'launch_events_quiz.py'
    )
    spec = importlib.util.spec_from_file_location('launch_events_quiz', src_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_process_exit_event():
    """测试: 进程退出后触发的事件"""
    module = _load_quiz_module()
    assert module.answer_process_exit_event == 'OnProcessExit', \
        '当进程退出后，OnProcessExit 事件会被触发'


def test_register_handler():
    """测试: 注册事件处理器的类"""
    module = _load_quiz_module()
    assert module.answer_register_handler == 'RegisterEventHandler', \
        'RegisterEventHandler 用于注册事件处理器'


def test_chained_launch():
    """测试: launch 是否支持链式启动"""
    module = _load_quiz_module()
    assert module.answer_chained_launch == 'yes', \
        'launch 支持"先启动 A，A 退出后再启动 B"的模式（通过 OnProcessExit 实现）'


def test_shutdown_action():
    """测试: 关闭整个 launch 的 action"""
    module = _load_quiz_module()
    assert module.answer_shutdown_action == 'Shutdown', \
        'Shutdown action 可以在事件触发时关闭整个 launch'


def test_exit_code_access():
    """测试: OnProcessExit 是否可以访问退出码"""
    module = _load_quiz_module()
    assert module.answer_exit_code_access == 'yes', \
        'OnProcessExit 事件可以通过 event.returncode 访问进程的退出码'
