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
#include "../src/frame_chain.cpp"

class FrameChainTest : public ::testing::Test {
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

  // Helper: create student node + listener, spin until transforms arrive, return buffer
  std::shared_ptr<tf2_ros::Buffer> spin_and_get_buffer(
    std::shared_ptr<FrameChainNode> node,
    const std::string & target_frame,
    const std::string & source_frame)
  {
    auto listener_node = std::make_shared<rclcpp::Node>("test_listener_fc");
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

TEST_F(FrameChainTest, CanCreateNode) {
  auto node = std::make_shared<FrameChainNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(FrameChainTest, MapToOdomTransformExists) {
  auto node = std::make_shared<FrameChainNode>();
  auto buffer = spin_and_get_buffer(node, "map", "odom");

  try {
    auto t = buffer->lookupTransform("map", "odom", tf2::TimePointZero);
    EXPECT_EQ(t.header.frame_id, "map");
    EXPECT_EQ(t.child_frame_id, "odom");
    EXPECT_DOUBLE_EQ(t.transform.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.0);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Transform map -> odom not found: " << ex.what();
  }
}

TEST_F(FrameChainTest, OdomToBaseLinkTransformExists) {
  auto node = std::make_shared<FrameChainNode>();
  auto buffer = spin_and_get_buffer(node, "odom", "base_link");

  try {
    auto t = buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
    EXPECT_EQ(t.header.frame_id, "odom");
    EXPECT_EQ(t.child_frame_id, "base_link");
    EXPECT_DOUBLE_EQ(t.transform.translation.x, 0.5);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.0);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Transform odom -> base_link not found: " << ex.what();
  }
}

TEST_F(FrameChainTest, BaseLinkToSensorLinkTransformExists) {
  auto node = std::make_shared<FrameChainNode>();
  auto buffer = spin_and_get_buffer(node, "base_link", "sensor_link");

  try {
    auto t = buffer->lookupTransform("base_link", "sensor_link", tf2::TimePointZero);
    EXPECT_EQ(t.header.frame_id, "base_link");
    EXPECT_EQ(t.child_frame_id, "sensor_link");
    EXPECT_DOUBLE_EQ(t.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.3);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Transform base_link -> sensor_link not found: " << ex.what();
  }
}

TEST_F(FrameChainTest, CanLookupChainedTransform) {
  auto node = std::make_shared<FrameChainNode>();
  auto buffer = spin_and_get_buffer(node, "map", "sensor_link");

  // Query chained transform: map -> sensor_link
  // Expected: (1.0 + 0.5 + 0.0, 0.0, 0.0 + 0.0 + 0.3) = (1.5, 0.0, 0.3)
  try {
    auto result = buffer->lookupTransform("map", "sensor_link", tf2::TimePointZero);
    EXPECT_NEAR(result.transform.translation.x, 1.5, 1e-6);
    EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.z, 0.3, 1e-6);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "Chained transform map -> sensor_link not found: " << ex.what();
  }
}
