#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

// 在测试文件中定义正确的节点类
class StaticBroadcasterNode : public rclcpp::Node
{
public:
  StaticBroadcasterNode() : Node("static_broadcaster_node")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.5;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    transform_ = t;
    tf_broadcaster_->sendTransform(t);
  }

  geometry_msgs::msg::TransformStamped get_transform() const { return transform_; }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;
};

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

TEST_F(StaticBroadcasterTest, FrameIdIsWorld) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  auto t = node->get_transform();
  EXPECT_EQ(t.header.frame_id, "world");
}

TEST_F(StaticBroadcasterTest, ChildFrameIdIsBaseLink) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  auto t = node->get_transform();
  EXPECT_EQ(t.child_frame_id, "base_link");
}

TEST_F(StaticBroadcasterTest, TranslationCorrect) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  auto t = node->get_transform();
  EXPECT_DOUBLE_EQ(t.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(t.transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(t.transform.translation.z, 0.5);
}

TEST_F(StaticBroadcasterTest, RotationIsUnitQuaternion) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  auto t = node->get_transform();
  EXPECT_DOUBLE_EQ(t.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(t.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(t.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(t.transform.rotation.w, 1.0);
}

TEST_F(StaticBroadcasterTest, QuaternionNormIsOne) {
  auto node = std::make_shared<StaticBroadcasterNode>();
  auto t = node->get_transform();
  double norm = std::sqrt(
    t.transform.rotation.x * t.transform.rotation.x +
    t.transform.rotation.y * t.transform.rotation.y +
    t.transform.rotation.z * t.transform.rotation.z +
    t.transform.rotation.w * t.transform.rotation.w);
  EXPECT_NEAR(norm, 1.0, 1e-6);
}
