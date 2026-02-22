#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <cmath>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/dynamic_broadcaster.cpp"

class DynamicBroadcasterTest : public ::testing::Test {
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

TEST_F(DynamicBroadcasterTest, CanCreateNode) {
  auto node = std::make_shared<DynamicBroadcasterNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(DynamicBroadcasterTest, ParentFrameIsBaseLink) {
  auto node = std::make_shared<DynamicBroadcasterNode>();
  EXPECT_EQ(node->get_parent_frame(), "base_link");
}

TEST_F(DynamicBroadcasterTest, ChildFrameIsSensorLink) {
  auto node = std::make_shared<DynamicBroadcasterNode>();
  EXPECT_EQ(node->get_child_frame(), "sensor_link");
}

TEST_F(DynamicBroadcasterTest, TimerPublishesTransform) {
  auto node = std::make_shared<DynamicBroadcasterNode>();

  // Create a listener node with buffer to pick up the dynamic transforms
  auto listener_node = std::make_shared<rclcpp::Node>("test_listener_node");
  auto buffer = std::make_shared<tf2_ros::Buffer>(listener_node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, listener_node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listener_node);

  // Spin to let the timer fire and transforms propagate
  auto start = std::chrono::steady_clock::now();
  bool found = false;
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    executor.spin_some(std::chrono::milliseconds(50));
    try {
      auto t = buffer->lookupTransform("base_link", "sensor_link", tf2::TimePointZero);
      found = true;
      break;
    } catch (const tf2::TransformException &) {
      // Transform not yet available, keep spinning
    }
  }
  ASSERT_TRUE(found) << "Dynamic transform base_link -> sensor_link was not received within timeout";
}

TEST_F(DynamicBroadcasterTest, TransformHasValidTimestampAndFrames) {
  auto node = std::make_shared<DynamicBroadcasterNode>();

  auto listener_node = std::make_shared<rclcpp::Node>("test_listener_node_ts");
  auto buffer = std::make_shared<tf2_ros::Buffer>(listener_node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, listener_node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listener_node);

  auto start = std::chrono::steady_clock::now();
  bool found = false;
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    executor.spin_some(std::chrono::milliseconds(50));
    try {
      auto t = buffer->lookupTransform("base_link", "sensor_link", tf2::TimePointZero);
      // Verify frame names
      EXPECT_EQ(t.header.frame_id, "base_link");
      EXPECT_EQ(t.child_frame_id, "sensor_link");
      // Verify timestamp is non-zero (meaning this->now() was used)
      EXPECT_GT(t.header.stamp.sec + t.header.stamp.nanosec, 0u);
      // Verify z translation is 0.3
      EXPECT_NEAR(t.transform.translation.z, 0.3, 1e-6);
      found = true;
      break;
    } catch (const tf2::TransformException &) {
      // Transform not yet available, keep spinning
    }
  }
  ASSERT_TRUE(found) << "Dynamic transform not received within timeout for validation";
}
