#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

class CoordinateTransformTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_coordinate_transform_node");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    // 设置已知变换: map -> sensor_link，平移 (2.0, 0.0, 0.0)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_->now();
    t.header.frame_id = "map";
    t.child_frame_id = "sensor_link";
    t.transform.translation.x = 2.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0;
    buffer_->setTransform(t, "test_authority", true);
  }

  void TearDown() override {
    node_.reset();
    buffer_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
};

TEST_F(CoordinateTransformTest, CanLookupTransform) {
  // 验证可以查询 sensor_link -> map 的变换
  ASSERT_NO_THROW({
    auto transform = buffer_->lookupTransform(
      "map", "sensor_link", tf2::TimePointZero);
  });
}

TEST_F(CoordinateTransformTest, LookupTransformCorrect) {
  // 验证查询到的变换平移值正确
  auto transform = buffer_->lookupTransform(
    "map", "sensor_link", tf2::TimePointZero);
  EXPECT_NEAR(transform.transform.translation.x, 2.0, 1e-6);
  EXPECT_NEAR(transform.transform.translation.y, 0.0, 1e-6);
  EXPECT_NEAR(transform.transform.translation.z, 0.0, 1e-6);
}

TEST_F(CoordinateTransformTest, DoTransformPointCorrect) {
  // 在 sensor_link 帧中创建点 (1.0, 0.0, 0.0)
  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = "sensor_link";
  point_in.header.stamp = node_->now();
  point_in.point.x = 1.0;
  point_in.point.y = 0.0;
  point_in.point.z = 0.0;

  // 获取变换
  auto transform = buffer_->lookupTransform(
    "map", "sensor_link", tf2::TimePointZero);

  // 执行变换
  geometry_msgs::msg::PointStamped point_out;
  tf2::doTransform(point_in, point_out, transform);

  // 验证: sensor_link 中 (1,0,0) + 平移 (2,0,0) = map 中 (3,0,0)
  EXPECT_NEAR(point_out.point.x, 3.0, 1e-6);
  EXPECT_NEAR(point_out.point.y, 0.0, 1e-6);
  EXPECT_NEAR(point_out.point.z, 0.0, 1e-6);
}

TEST_F(CoordinateTransformTest, TransformedPointFrameIsTarget) {
  // 验证变换后的点的 frame_id 是目标帧
  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = "sensor_link";
  point_in.header.stamp = node_->now();
  point_in.point.x = 1.0;
  point_in.point.y = 0.0;
  point_in.point.z = 0.0;

  auto transform = buffer_->lookupTransform(
    "map", "sensor_link", tf2::TimePointZero);

  geometry_msgs::msg::PointStamped point_out;
  tf2::doTransform(point_in, point_out, transform);

  EXPECT_EQ(point_out.header.frame_id, "map");
}

TEST_F(CoordinateTransformTest, DoTransformWithOffset) {
  // 添加额外变换测试: map -> sensor_link 平移 (2,3,1)
  geometry_msgs::msg::TransformStamped t2;
  t2.header.stamp = node_->now();
  t2.header.frame_id = "map";
  t2.child_frame_id = "sensor2_link";
  t2.transform.translation.x = 2.0;
  t2.transform.translation.y = 3.0;
  t2.transform.translation.z = 1.0;
  t2.transform.rotation.w = 1.0;
  buffer_->setTransform(t2, "test_authority", true);

  // 在 sensor2_link 中的点 (0.5, -1.0, 0.0)
  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = "sensor2_link";
  point_in.header.stamp = node_->now();
  point_in.point.x = 0.5;
  point_in.point.y = -1.0;
  point_in.point.z = 0.0;

  auto transform = buffer_->lookupTransform(
    "map", "sensor2_link", tf2::TimePointZero);

  geometry_msgs::msg::PointStamped point_out;
  tf2::doTransform(point_in, point_out, transform);

  // 验证: (0.5, -1.0, 0.0) + (2.0, 3.0, 1.0) = (2.5, 2.0, 1.0)
  EXPECT_NEAR(point_out.point.x, 2.5, 1e-6);
  EXPECT_NEAR(point_out.point.y, 2.0, 1e-6);
  EXPECT_NEAR(point_out.point.z, 1.0, 1e-6);
}
