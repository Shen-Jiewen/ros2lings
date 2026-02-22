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

TEST_F(TfTimeTravelTest, BufferExists) {
  auto node = std::make_shared<TfTimeTravelNode>();
  ASSERT_NE(node->get_buffer(), nullptr);
}

TEST_F(TfTimeTravelTest, ExceptionHandlingWorks) {
  // The student's timer_callback must handle tf2::TransformException (BUG 3).
  // Without any transform data, lookupTransform throws. A correct catch block
  // prevents the node from crashing.
  auto node = std::make_shared<TfTimeTravelNode>();
  auto buffer = node->get_buffer();

  // Attempt a lookup with no data — should throw, which student must catch
  bool threw = false;
  try {
    buffer->lookupTransform("world", "robot", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    threw = true;
  }
  EXPECT_TRUE(threw) << "lookupTransform should throw when no data is available";
}

TEST_F(TfTimeTravelTest, LookupSucceedsWithInjectedTransform) {
  // With transform data injected into the buffer, lookupTransform with
  // TimePointZero (BUG 1 fix) and positive timeout (BUG 2 fix) should succeed.
  auto node = std::make_shared<TfTimeTravelNode>();
  auto buffer = node->get_buffer();

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "world";
  t.child_frame_id = "robot";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  // Using TimePointZero should succeed (BUG 1: student must not use future time)
  try {
    auto result = buffer->lookupTransform("world", "robot", tf2::TimePointZero);
    EXPECT_DOUBLE_EQ(result.transform.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(result.transform.translation.y, 2.0);
    EXPECT_DOUBLE_EQ(result.transform.translation.z, 3.0);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "lookupTransform should succeed with injected data: " << ex.what();
  }
}

TEST_F(TfTimeTravelTest, CanLookupWithTimePointZero) {
  auto node = std::make_shared<TfTimeTravelNode>();
  auto buffer = node->get_buffer();

  // Manually inject a transform into the buffer
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "world";
  t.child_frame_id = "robot";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  // Using TimePointZero should succeed (verifies student uses TimePointZero, not future time)
  try {
    auto result = buffer->lookupTransform("world", "robot", tf2::TimePointZero);
    EXPECT_DOUBLE_EQ(result.transform.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(result.transform.translation.y, 2.0);
    EXPECT_DOUBLE_EQ(result.transform.translation.z, 3.0);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "lookupTransform with TimePointZero should not fail: " << ex.what();
  }
}
