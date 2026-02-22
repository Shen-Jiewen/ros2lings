#!/usr/bin/env python3
"""Tests for hello_node_py exercise.

These tests import the student's source file and verify that it:
  1. Defines a callable main() function.
  2. Calls rclpy.init() before creating a node.
  3. Creates a Node named 'hello_node_py'.
  4. Calls rclpy.spin_once on the node.
  5. The main() function runs without error.
"""
import importlib.util
import os

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _student_source_path():
    test_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(test_dir, '..', 'src', 'hello_node_py.py')


def _read_student_source():
    """Return the raw source text of the student file."""
    with open(_student_source_path(), 'r') as f:
        return f.read()


def _load_student_module():
    """Load the student's source file as a module."""
    src_path = _student_source_path()
    spec = importlib.util.spec_from_file_location('student_module', src_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_main_function_exists():
    """The student module must define a callable main() function."""
    mod = _load_student_module()
    assert hasattr(mod, 'main'), (
        'The student module must define a main() function.'
    )
    assert callable(mod.main), 'main must be callable.'


def test_init_before_node_creation():
    """rclpy.init() must be called before Node() is constructed.

    Inspects the source code to verify the call order inside main().
    """
    source = _read_student_source()

    # Walk the lines inside the main() function body.
    lines = source.split('\n')
    in_main = False
    init_line = None
    node_line = None
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith('def main'):
            in_main = True
            continue
        if in_main:
            # Stop if we hit another top-level def/class
            if stripped and not stripped.startswith('#') and not stripped.startswith("'") \
               and not stripped.startswith('"'):
                if line and not line[0].isspace():
                    break
            if 'rclpy.init()' in stripped and not stripped.startswith('#'):
                if init_line is None:
                    init_line = i
            if 'Node(' in stripped and not stripped.startswith('#'):
                if node_line is None:
                    node_line = i

    assert init_line is not None, (
        'Could not find rclpy.init() call inside main().'
    )
    assert node_line is not None, (
        'Could not find Node(...) creation inside main().'
    )
    assert init_line < node_line, (
        'rclpy.init() must be called before creating the Node. '
        f'Found init at line {init_line + 1}, Node at line {node_line + 1}.'
    )


def test_node_name_is_hello_node_py():
    """The student must create a Node named 'hello_node_py'."""
    source = _read_student_source()
    assert "'hello_node_py'" in source or '"hello_node_py"' in source, (
        "The node name must be 'hello_node_py'."
    )


def test_spin_once_is_called():
    """main() must call rclpy.spin_once on the node."""
    source = _read_student_source()
    assert 'spin_once' in source, (
        'main() must call rclpy.spin_once(node, ...) to spin the node at least once.'
    )


def test_main_runs_without_error():
    """Calling main() should execute successfully end-to-end."""
    mod = _load_student_module()
    # main() manages its own rclpy.init / shutdown, so just call it.
    try:
        mod.main()
    except Exception as exc:
        pytest.fail(f'main() raised an exception: {exc}')
