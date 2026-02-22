#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <vector>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/multi_robot_tf.cpp"

class MultiRobotTfTest : public ::testing::Test {
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

  // Helper: create student node + listener, spin until transform arrives, return buffer
  std::shared_ptr<tf2_ros::Buffer> spin_and_get_buffer(
    std::shared_ptr<MultiRobotTfNode> node,
    const std::string & target_frame,
    const std::string & source_frame)
  {
    auto listener_node = std::make_shared<rclcpp::Node>("test_listener_mr");
    auto buffer = std::make_shared<tf2_ros::Buffer>(listener_node->get_clock());
    auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, listener_node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(listener_node);

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
      executor.spin_some(std::chrono::milliseconds(50));
      try {
        buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        return buffer;
      } catch (const tf2::TransformException &) {
        // Not yet available
      }
    }
    return buffer;
  }
};

TEST_F(MultiRobotTfTest, CanCreateNode) {
  auto node = std::make_shared<MultiRobotTfNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(MultiRobotTfTest, Robot1BaseLinkTransformCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto buffer = spin_and_get_buffer(node, "world", "robot1/base_link");

  try {
    auto t = buffer->lookupTransform("world", "robot1/base_link", tf2::TimePointZero);
    EXPECT_EQ(t.header.frame_id, "world");
    EXPECT_EQ(t.child_frame_id, "robot1/base_link");
    EXPECT_DOUBLE_EQ(t.transform.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.0);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Transform world -> robot1/base_link not found: " << ex.what();
  }
}

TEST_F(MultiRobotTfTest, Robot1SensorTransformCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto buffer = spin_and_get_buffer(node, "robot1/base_link", "robot1/sensor");

  try {
    auto t = buffer->lookupTransform("robot1/base_link", "robot1/sensor", tf2::TimePointZero);
    EXPECT_EQ(t.header.frame_id, "robot1/base_link");
    EXPECT_EQ(t.child_frame_id, "robot1/sensor");
    EXPECT_DOUBLE_EQ(t.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.5);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Transform robot1/base_link -> robot1/sensor not found: " << ex.what();
  }
}

TEST_F(MultiRobotTfTest, Robot2BaseLinkTransformCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto buffer = spin_and_get_buffer(node, "world", "robot2/base_link");

  try {
    auto t = buffer->lookupTransform("world", "robot2/base_link", tf2::TimePointZero);
    EXPECT_EQ(t.header.frame_id, "world");
    EXPECT_EQ(t.child_frame_id, "robot2/base_link");
    EXPECT_DOUBLE_EQ(t.transform.translation.x, -1.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.0);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Transform world -> robot2/base_link not found: " << ex.what();
  }
}

TEST_F(MultiRobotTfTest, Robot2SensorTransformCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto buffer = spin_and_get_buffer(node, "robot2/base_link", "robot2/sensor");

  try {
    auto t = buffer->lookupTransform("robot2/base_link", "robot2/sensor", tf2::TimePointZero);
    EXPECT_EQ(t.header.frame_id, "robot2/base_link");
    EXPECT_EQ(t.child_frame_id, "robot2/sensor");
    EXPECT_DOUBLE_EQ(t.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.5);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Transform robot2/base_link -> robot2/sensor not found: " << ex.what();
  }
}

TEST_F(MultiRobotTfTest, CanLookupRobot1SensorFromWorld) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto buffer = spin_and_get_buffer(node, "world", "robot1/sensor");

  // Chained transform: world -> robot1/sensor
  // Expected: (1.0, 0.0, 0.0) + (0.0, 0.0, 0.5) = (1.0, 0.0, 0.5)
  try {
    auto result = buffer->lookupTransform("world", "robot1/sensor", tf2::TimePointZero);
    EXPECT_NEAR(result.transform.translation.x, 1.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.z, 0.5, 1e-6);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Chained transform world -> robot1/sensor not found: " << ex.what();
  }
}

TEST_F(MultiRobotTfTest, CanLookupRobot2SensorFromWorld) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto buffer = spin_and_get_buffer(node, "world", "robot2/sensor");

  // Chained transform: world -> robot2/sensor
  // Expected: (-1.0, 0.0, 0.0) + (0.0, 0.0, 0.5) = (-1.0, 0.0, 0.5)
  try {
    auto result = buffer->lookupTransform("world", "robot2/sensor", tf2::TimePointZero);
    EXPECT_NEAR(result.transform.translation.x, -1.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.z, 0.5, 1e-6);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Chained transform world -> robot2/sensor not found: " << ex.what();
  }
}
