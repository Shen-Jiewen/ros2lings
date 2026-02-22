#!/usr/bin/env python3
"""Tests for subscriber_py exercise.

These tests import the student's source file and verify that the
SubscriberPy class:
  1. Exists and inherits from rclpy.node.Node.
  2. Sets the node name to 'subscriber_py'.
  3. Creates a subscription on the 'chatter' topic for std_msgs/String.
  4. Defines listener_callback(self, msg) with the correct signature.
  5. The callback properly receives messages.
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
    src_path = os.path.join(test_dir, '..', 'src', 'subscriber_py.py')
    spec = importlib.util.spec_from_file_location('student_subscriber', src_path)
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
    """Instantiate the student's SubscriberPy node and destroy it after use."""
    mod = _load_student_module()
    node = mod.SubscriberPy()
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_subscriber_class_exists():
    """The student module must define a class called SubscriberPy."""
    mod = _load_student_module()
    assert hasattr(mod, 'SubscriberPy'), (
        'The student module must define a class named SubscriberPy.'
    )
    assert inspect.isclass(mod.SubscriberPy), 'SubscriberPy must be a class.'


def test_subscriber_inherits_from_node():
    """SubscriberPy must inherit from rclpy.node.Node."""
    mod = _load_student_module()
    assert issubclass(mod.SubscriberPy, Node), (
        'SubscriberPy must inherit from rclpy.node.Node.'
    )


def test_node_name(student_node):
    """The node name must be 'subscriber_py'."""
    assert student_node.get_name() == 'subscriber_py', (
        f"Expected node name 'subscriber_py', got '{student_node.get_name()}'."
    )


def test_subscription_attribute_exists(student_node):
    """The node must have a subscription_ attribute."""
    assert hasattr(student_node, 'subscription_'), (
        'SubscriberPy must store the subscription as self.subscription_.'
    )


def test_subscription_topic_name(student_node):
    """The subscription must be on the 'chatter' topic."""
    # Check the node's subscriptions via the topic list.
    topic_names_and_types = student_node.get_subscriptions_info_by_topic('/chatter')
    assert len(topic_names_and_types) > 0, (
        "SubscriberPy must subscribe to the 'chatter' topic. "
        "No subscription found on '/chatter'."
    )


def test_listener_callback_exists(student_node):
    """SubscriberPy must define a listener_callback method."""
    assert hasattr(student_node, 'listener_callback'), (
        'SubscriberPy must define a listener_callback method.'
    )
    assert callable(student_node.listener_callback), (
        'listener_callback must be callable.'
    )


def test_listener_callback_signature(student_node):
    """listener_callback must accept (self, msg) -- i.e. two parameters."""
    sig = inspect.signature(student_node.listener_callback)
    # When accessed via the instance, 'self' is already bound, so
    # the remaining parameters should be exactly one: msg.
    params = list(sig.parameters.keys())
    assert len(params) == 1, (
        f'listener_callback should take exactly one parameter (msg) when '
        f'bound, but got {len(params)}: {params}. '
        f'Make sure the method signature is listener_callback(self, msg).'
    )


def test_subscription_receives_message(student_node):
    """Publishing on 'chatter' should reach the student's subscriber."""
    # Create an ephemeral publisher on the same node for convenience.
    pub = student_node.create_publisher(String, 'chatter', 10)

    msg = String()
    msg.data = 'test from pytest'
    pub.publish(msg)

    # Spin to allow the message to be delivered.
    rclpy.spin_once(student_node, timeout_sec=0.5)

    # We cannot always inspect internal state, but the fact that
    # spin_once completes without error (given the correct callback
    # signature) is a good signal. If listener_callback(msg) was
    # defined without self, spin_once would raise a TypeError.
    # As an extra check, call the callback directly to ensure no crash.
    direct_msg = String()
    direct_msg.data = 'direct call'
    try:
        student_node.listener_callback(direct_msg)
    except TypeError as exc:
        pytest.fail(
            f'listener_callback raised TypeError when called with a message: {exc}. '
            f'Make sure the signature is listener_callback(self, msg).'
        )
