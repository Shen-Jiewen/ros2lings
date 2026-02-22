#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/tf_listener.cpp"

class TfListenerTest : public ::testing::Test {
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

TEST_F(TfListenerTest, CanCreateNode) {
  auto node = std::make_shared<TfListenerNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(TfListenerTest, BufferExists) {
  auto node = std::make_shared<TfListenerNode>();
  ASSERT_NE(node->get_buffer(), nullptr);
}

TEST_F(TfListenerTest, TargetFrameIsBaseLink) {
  auto node = std::make_shared<TfListenerNode>();
  EXPECT_EQ(node->get_target_frame(), "base_link");
}

TEST_F(TfListenerTest, SourceFrameIsSensorLink) {
  auto node = std::make_shared<TfListenerNode>();
  EXPECT_EQ(node->get_source_frame(), "sensor_link");
}

TEST_F(TfListenerTest, LookupTransformHandlesException) {
  auto node = std::make_shared<TfListenerNode>();
  auto buffer = node->get_buffer();
  // Without any transforms in the buffer, lookupTransform should throw
  // The student's timer_callback should handle this via try-catch
  // Verify the buffer lookup throws when no data is available
  EXPECT_THROW(
    buffer->lookupTransform("base_link", "sensor_link", tf2::TimePointZero),
    tf2::TransformException);
}

TEST_F(TfListenerTest, LookupTransformFrameOrder) {
  auto node = std::make_shared<TfListenerNode>();
  auto buffer = node->get_buffer();

  // Manually inject a transform into the buffer to verify frame order
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "sensor_link";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  // Verify correct lookup order: lookupTransform(target_frame, source_frame, time)
  // The student should use ("base_link", "sensor_link", ...)
  try {
    auto result = buffer->lookupTransform(
      node->get_target_frame(), node->get_source_frame(), tf2::TimePointZero);
    EXPECT_EQ(result.header.frame_id, "base_link");
    EXPECT_EQ(result.child_frame_id, "sensor_link");
  } catch (const tf2::TransformException & ex) {
    FAIL() << "lookupTransform should not throw with valid data: " << ex.what();
  }
}
