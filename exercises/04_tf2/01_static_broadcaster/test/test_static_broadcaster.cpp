#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/static_broadcaster.cpp"

class StaticBroadcasterTest : public ::testing::Test {
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

TEST_F(StaticBroadcasterTest, CanCreateNode) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(StaticBroadcasterTest, ParentFrameIsWorld) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  EXPECT_EQ(node->get_parent_frame(), "world");
}

TEST_F(StaticBroadcasterTest, ChildFrameIsBaseLink) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  EXPECT_EQ(node->get_child_frame(), "base_link");
}

TEST_F(StaticBroadcasterTest, BroadcastsCorrectTransform) {
  auto node = std::make_shared<StaticBroadcasterNode>();

  // Create a listener node with buffer to pick up the static transform
  auto listener_node = std::make_shared<rclcpp::Node>("test_listener_node");
  auto buffer = std::make_shared<tf2_ros::Buffer>(listener_node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, listener_node);

  // Spin both nodes briefly so the static transform is received
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listener_node);

  auto start = std::chrono::steady_clock::now();
  bool found = false;
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    executor.spin_some(std::chrono::milliseconds(50));
    try {
      auto t = buffer->lookupTransform("world", "base_link", tf2::TimePointZero);
      // Verify translation
      EXPECT_DOUBLE_EQ(t.transform.translation.x, 1.0);
      EXPECT_DOUBLE_EQ(t.transform.translation.y, 2.0);
      EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.5);
      // Verify rotation is unit quaternion
      EXPECT_DOUBLE_EQ(t.transform.rotation.x, 0.0);
      EXPECT_DOUBLE_EQ(t.transform.rotation.y, 0.0);
      EXPECT_DOUBLE_EQ(t.transform.rotation.z, 0.0);
      EXPECT_DOUBLE_EQ(t.transform.rotation.w, 1.0);
      found = true;
      break;
    } catch (const tf2::TransformException &) {
      // Transform not yet available, keep spinning
    }
  }
  ASSERT_TRUE(found) << "Static transform world -> base_link was not received within timeout";
}

TEST_F(StaticBroadcasterTest, QuaternionNormIsOne) {
  auto node = std::make_shared<StaticBroadcasterNode>();

  auto listener_node = std::make_shared<rclcpp::Node>("test_listener_node_quat");
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
      auto t = buffer->lookupTransform("world", "base_link", tf2::TimePointZero);
      double norm = std::sqrt(
        t.transform.rotation.x * t.transform.rotation.x +
        t.transform.rotation.y * t.transform.rotation.y +
        t.transform.rotation.z * t.transform.rotation.z +
        t.transform.rotation.w * t.transform.rotation.w);
      EXPECT_NEAR(norm, 1.0, 1e-6);
      found = true;
      break;
    } catch (const tf2::TransformException &) {
      // Transform not yet available, keep spinning
    }
  }
  ASSERT_TRUE(found) << "Static transform not received within timeout for quaternion norm check";
}
