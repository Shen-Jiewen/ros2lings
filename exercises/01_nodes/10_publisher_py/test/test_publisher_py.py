#!/usr/bin/env python3
"""Tests for publisher_py exercise.

These tests import the student's source file and verify that the
PublisherPy class:
  1. Exists and inherits from rclpy.node.Node.
  2. Sets the node name to 'publisher_py'.
  3. Creates a publisher on the 'chatter' topic for std_msgs/String.
  4. Creates a timer with a bound callback method.
  5. The timer callback publishes a message via the publisher.
"""
import importlib.util
import inspect
import os

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Helper: load student module
# ---------------------------------------------------------------------------

def _load_student_module():
    """Load the student's source file as a module."""
    test_dir = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(test_dir, '..', 'src', 'publisher_py.py')
    spec = importlib.util.spec_from_file_location('student_publisher', src_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    """Initialise / shutdown rclpy once for the whole test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def student_node():
    """Instantiate the student's PublisherPy node and destroy it after use."""
    mod = _load_student_module()
    node = mod.PublisherPy()
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_publisher_class_exists():
    """The student module must define a class called PublisherPy."""
    mod = _load_student_module()
    assert hasattr(mod, 'PublisherPy'), (
        'The student module must define a class named PublisherPy.'
    )
    assert inspect.isclass(mod.PublisherPy), 'PublisherPy must be a class.'


def test_publisher_inherits_from_node():
    """PublisherPy must inherit from rclpy.node.Node."""
    mod = _load_student_module()
    assert issubclass(mod.PublisherPy, Node), (
        'PublisherPy must inherit from rclpy.node.Node.'
    )


def test_node_name(student_node):
    """The node name must be 'publisher_py'."""
    assert student_node.get_name() == 'publisher_py', (
        f"Expected node name 'publisher_py', got '{student_node.get_name()}'."
    )


def test_publisher_attribute_exists(student_node):
    """The node must have a publisher_ attribute."""
    assert hasattr(student_node, 'publisher_'), (
        'PublisherPy must store the publisher as self.publisher_.'
    )


def test_publisher_topic_name(student_node):
    """The publisher must be on the 'chatter' topic."""
    pub = student_node.publisher_
    assert pub.topic_name == '/chatter' or pub.topic_name == 'chatter', (
        f"Publisher topic must be 'chatter', got '{pub.topic_name}'."
    )


def test_timer_attribute_exists(student_node):
    """The node must have a timer_ attribute."""
    assert hasattr(student_node, 'timer_'), (
        'PublisherPy must store the timer as self.timer_.'
    )


def test_timer_callback_is_callable(student_node):
    """The timer must be bound to a callable callback (not a string)."""
    assert student_node.timer_ is not None, 'timer_ must not be None.'
    # The timer was successfully created, meaning the callback was valid.
    # Also verify the method itself exists and is callable.
    assert hasattr(student_node, 'timer_callback'), (
        'PublisherPy must define a timer_callback method.'
    )
    assert callable(student_node.timer_callback), (
        'timer_callback must be callable.'
    )


def test_timer_callback_publishes(student_node):
    """Calling timer_callback() must publish a message on the publisher."""
    # Create a subscriber to capture the published message.
    received = []
    sub = student_node.create_subscription(
        String,
        'chatter',
        lambda msg: received.append(msg.data),
        10,
    )

    # Invoke the callback directly.
    student_node.timer_callback()

    # Spin to allow the message to be delivered intra-process.
    rclpy.spin_once(student_node, timeout_sec=0.5)

    assert len(received) > 0, (
        'timer_callback() must publish a message via self.publisher_. '
        'No message was received on the chatter topic.'
    )
    sub  # prevent unused-variable lint
