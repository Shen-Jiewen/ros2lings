#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <chrono>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/tf_time_travel.cpp"

using namespace std::chrono_literals;

class TfTimeTravelTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(TfTimeTravelTest, CanCreateNode) {
  auto node = std::make_shared<TfTimeTravelNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(TfTimeTravelTest, TimerCallbackSucceedsWithTransformData) {
  // This test verifies the student's timer_callback actually works:
  // 1. Inject a transform into the buffer
  // 2. Spin the node so the timer fires and timer_callback executes
  // 3. Check that lookup_succeeded_ becomes true
  //
  // With the buggy code (future timestamp), lookupTransform always fails
  // because no transform data exists 10 seconds in the future.
  // Only the fixed code (tf2::TimePointZero) will find the injected data.
  auto node = std::make_shared<TfTimeTravelNode>();
  auto buffer = node->get_buffer();

  // Inject a NON-STATIC transform at the current time.
  // Using false (non-static) is critical: static transforms are available at
  // ALL times (including future), which would let the buggy code
  // (lookupTransform at now+10s) succeed.  Non-static transforms are only
  // valid around their stamped time, so querying at a future timestamp will
  // throw ExtrapolationException — exactly what we need to detect Bug 1.
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "world";
  t.child_frame_id = "robot";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", false);

  // Spin the node so the timer (500ms) fires at least once
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < 2s) {
    executor.spin_some(50ms);
    if (node->get_lookup_succeeded()) {
      break;
    }
  }

  ASSERT_GT(node->get_callback_count(), 0)
    << "timer_callback should have been called at least once";
  EXPECT_TRUE(node->get_lookup_succeeded())
    << "timer_callback's lookupTransform should succeed with injected data. "
       "Make sure you use tf2::TimePointZero (not a future timestamp) and "
       "a non-zero timeout.";
}

TEST_F(TfTimeTravelTest, CallbackHandlesNoData) {
  // Without any transform data, timer_callback should not crash.
  // This verifies the student's exception handling (BUG 3).
  auto node = std::make_shared<TfTimeTravelNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin briefly — timer fires, lookupTransform throws, student must catch
  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < 1s) {
    executor.spin_some(50ms);
    if (node->get_callback_count() > 0) {
      break;
    }
  }

  ASSERT_GT(node->get_callback_count(), 0)
    << "timer_callback should have been called at least once";
  // If we get here without crashing, exception handling works
  EXPECT_FALSE(node->get_lookup_succeeded())
    << "lookup should fail when no transform data is available";
}
