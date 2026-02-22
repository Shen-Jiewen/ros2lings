// Test the student's CoordinateTransformNode class directly.
// The student source file is compiled into this test binary via CMakeLists.txt
// with ROS2LINGS_TEST defined (main is guarded out, get_buffer() is exposed).

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

// Pull in the student's class definition (all methods are inline / in-class).
// The student .cpp is also listed as a CMake source, but with ROS2LINGS_TEST
// defined and main guarded out it produces no linkable symbols, so there is
// no ODR or duplicate-symbol conflict.
#include "../src/coordinate_transform.cpp"

class CoordinateTransformTest : public ::testing::Test {
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

// -----------------------------------------------------------------
// RED when TODO 1 is not done: tf2_buffer_ is never initialised,
// so get_buffer() returns nullptr.
// -----------------------------------------------------------------

TEST_F(CoordinateTransformTest, NodeCanBeCreated) {
  auto node = std::make_shared<CoordinateTransformNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(CoordinateTransformTest, BufferIsInitialised) {
  auto node = std::make_shared<CoordinateTransformNode>();
  ASSERT_NE(node->get_buffer(), nullptr)
    << "tf2_buffer_ must be created in the constructor (TODO 1)";
}

// -----------------------------------------------------------------
// The remaining tests inject a known transform into the student's
// buffer and verify lookupTransform + doTransform behaviour.
// They exercise the same concepts the student must use in TODO 2-4.
// -----------------------------------------------------------------

TEST_F(CoordinateTransformTest, CanLookupTransform) {
  auto node = std::make_shared<CoordinateTransformNode>();
  auto buffer = node->get_buffer();
  ASSERT_NE(buffer, nullptr);

  // Inject a known static transform: map -> sensor_link, translation (2,0,0)
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "map";
  t.child_frame_id = "sensor_link";
  t.transform.translation.x = 2.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  ASSERT_NO_THROW({
    auto transform = buffer->lookupTransform(
      "map", "sensor_link", tf2::TimePointZero);
  });
}

TEST_F(CoordinateTransformTest, LookupTransformCorrect) {
  auto node = std::make_shared<CoordinateTransformNode>();
  auto buffer = node->get_buffer();
  ASSERT_NE(buffer, nullptr);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "map";
  t.child_frame_id = "sensor_link";
  t.transform.translation.x = 2.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  auto transform = buffer->lookupTransform(
    "map", "sensor_link", tf2::TimePointZero);
  EXPECT_NEAR(transform.transform.translation.x, 2.0, 1e-6);
  EXPECT_NEAR(transform.transform.translation.y, 0.0, 1e-6);
  EXPECT_NEAR(transform.transform.translation.z, 0.0, 1e-6);
}

TEST_F(CoordinateTransformTest, DoTransformPointCorrect) {
  auto node = std::make_shared<CoordinateTransformNode>();
  auto buffer = node->get_buffer();
  ASSERT_NE(buffer, nullptr);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "map";
  t.child_frame_id = "sensor_link";
  t.transform.translation.x = 2.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  // Point in sensor_link at (1,0,0)
  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = "sensor_link";
  point_in.header.stamp = node->now();
  point_in.point.x = 1.0;
  point_in.point.y = 0.0;
  point_in.point.z = 0.0;

  auto transform = buffer->lookupTransform(
    "map", "sensor_link", tf2::TimePointZero);

  geometry_msgs::msg::PointStamped point_out;
  tf2::doTransform(point_in, point_out, transform);

  // (1,0,0) + translation (2,0,0) = (3,0,0)
  EXPECT_NEAR(point_out.point.x, 3.0, 1e-6);
  EXPECT_NEAR(point_out.point.y, 0.0, 1e-6);
  EXPECT_NEAR(point_out.point.z, 0.0, 1e-6);
}

TEST_F(CoordinateTransformTest, TransformedPointFrameIsTarget) {
  auto node = std::make_shared<CoordinateTransformNode>();
  auto buffer = node->get_buffer();
  ASSERT_NE(buffer, nullptr);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "map";
  t.child_frame_id = "sensor_link";
  t.transform.translation.x = 2.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = "sensor_link";
  point_in.header.stamp = node->now();
  point_in.point.x = 1.0;
  point_in.point.y = 0.0;
  point_in.point.z = 0.0;

  auto transform = buffer->lookupTransform(
    "map", "sensor_link", tf2::TimePointZero);

  geometry_msgs::msg::PointStamped point_out;
  tf2::doTransform(point_in, point_out, transform);

  EXPECT_EQ(point_out.header.frame_id, "map");
}

TEST_F(CoordinateTransformTest, DoTransformWithOffset) {
  auto node = std::make_shared<CoordinateTransformNode>();
  auto buffer = node->get_buffer();
  ASSERT_NE(buffer, nullptr);

  // A second frame with a 3-axis offset
  geometry_msgs::msg::TransformStamped t2;
  t2.header.stamp = node->now();
  t2.header.frame_id = "map";
  t2.child_frame_id = "sensor2_link";
  t2.transform.translation.x = 2.0;
  t2.transform.translation.y = 3.0;
  t2.transform.translation.z = 1.0;
  t2.transform.rotation.w = 1.0;
  buffer->setTransform(t2, "test_authority", true);

  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = "sensor2_link";
  point_in.header.stamp = node->now();
  point_in.point.x = 0.5;
  point_in.point.y = -1.0;
  point_in.point.z = 0.0;

  auto transform = buffer->lookupTransform(
    "map", "sensor2_link", tf2::TimePointZero);

  geometry_msgs::msg::PointStamped point_out;
  tf2::doTransform(point_in, point_out, transform);

  // (0.5, -1.0, 0.0) + (2.0, 3.0, 1.0) = (2.5, 2.0, 1.0)
  EXPECT_NEAR(point_out.point.x, 2.5, 1e-6);
  EXPECT_NEAR(point_out.point.y, 2.0, 1e-6);
  EXPECT_NEAR(point_out.point.z, 1.0, 1e-6);
}
