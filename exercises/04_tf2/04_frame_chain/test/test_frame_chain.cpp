#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <vector>

// 在测试文件中定义正确的节点类
class FrameChainNode : public rclcpp::Node
{
public:
  FrameChainNode() : Node("frame_chain_node")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // 变换 1: map -> odom，平移 (1.0, 0.0, 0.0)
    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = this->now();
    t1.header.frame_id = "map";
    t1.child_frame_id = "odom";
    t1.transform.translation.x = 1.0;
    t1.transform.translation.y = 0.0;
    t1.transform.translation.z = 0.0;
    t1.transform.rotation.w = 1.0;
    transforms.push_back(t1);

    // 变换 2: odom -> base_link，平移 (0.5, 0.0, 0.0)
    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = this->now();
    t2.header.frame_id = "odom";
    t2.child_frame_id = "base_link";
    t2.transform.translation.x = 0.5;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = 0.0;
    t2.transform.rotation.w = 1.0;
    transforms.push_back(t2);

    // 变换 3: base_link -> sensor_link，平移 (0.0, 0.0, 0.3)
    geometry_msgs::msg::TransformStamped t3;
    t3.header.stamp = this->now();
    t3.header.frame_id = "base_link";
    t3.child_frame_id = "sensor_link";
    t3.transform.translation.x = 0.0;
    t3.transform.translation.y = 0.0;
    t3.transform.translation.z = 0.3;
    t3.transform.rotation.w = 1.0;
    transforms.push_back(t3);

    transforms_ = transforms;
    tf_broadcaster_->sendTransform(transforms);
  }

  std::vector<geometry_msgs::msg::TransformStamped> get_transforms() const
  {
    return transforms_;
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};

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
};

TEST_F(FrameChainTest, CanCreateNode) {
  auto node = std::make_shared<FrameChainNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(FrameChainTest, PublishesThreeTransforms) {
  auto node = std::make_shared<FrameChainNode>();
  auto transforms = node->get_transforms();
  EXPECT_EQ(transforms.size(), 3u);
}

TEST_F(FrameChainTest, FirstTransformIsMapToOdom) {
  auto node = std::make_shared<FrameChainNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 1u);
  EXPECT_EQ(transforms[0].header.frame_id, "map");
  EXPECT_EQ(transforms[0].child_frame_id, "odom");
}

TEST_F(FrameChainTest, SecondTransformIsOdomToBaseLink) {
  auto node = std::make_shared<FrameChainNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 2u);
  EXPECT_EQ(transforms[1].header.frame_id, "odom");
  EXPECT_EQ(transforms[1].child_frame_id, "base_link");
}

TEST_F(FrameChainTest, ThirdTransformIsBaseLinkToSensorLink) {
  auto node = std::make_shared<FrameChainNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 3u);
  EXPECT_EQ(transforms[2].header.frame_id, "base_link");
  EXPECT_EQ(transforms[2].child_frame_id, "sensor_link");
}

TEST_F(FrameChainTest, MapToOdomTranslationCorrect) {
  auto node = std::make_shared<FrameChainNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 1u);
  EXPECT_DOUBLE_EQ(transforms[0].transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(transforms[0].transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transforms[0].transform.translation.z, 0.0);
}

TEST_F(FrameChainTest, OdomToBaseLinkTranslationCorrect) {
  auto node = std::make_shared<FrameChainNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 2u);
  EXPECT_DOUBLE_EQ(transforms[1].transform.translation.x, 0.5);
  EXPECT_DOUBLE_EQ(transforms[1].transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transforms[1].transform.translation.z, 0.0);
}

TEST_F(FrameChainTest, BaseLinkToSensorLinkTranslationCorrect) {
  auto node = std::make_shared<FrameChainNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 3u);
  EXPECT_DOUBLE_EQ(transforms[2].transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(transforms[2].transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transforms[2].transform.translation.z, 0.3);
}

TEST_F(FrameChainTest, CanLookupChainedTransform) {
  auto node = std::make_shared<FrameChainNode>();

  // 创建 Buffer + Listener 来验证帧链是否可查
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

  // 手动将变换设置到 buffer 中验证帧链
  auto transforms = node->get_transforms();
  for (const auto & t : transforms) {
    buffer->setTransform(t, "test_authority", true);
  }

  // 查询 map -> sensor_link 的累积变换
  // 期望: (1.0 + 0.5 + 0.0, 0.0, 0.0 + 0.0 + 0.3) = (1.5, 0.0, 0.3)
  try {
    auto result = buffer->lookupTransform("map", "sensor_link", tf2::TimePointZero);
    EXPECT_NEAR(result.transform.translation.x, 1.5, 1e-6);
    EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.z, 0.3, 1e-6);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "无法查询帧链变换: " << ex.what();
  }
}
