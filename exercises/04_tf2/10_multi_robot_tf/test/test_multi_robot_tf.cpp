#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <vector>

// 在测试文件中定义正确的节点类
class MultiRobotTfNode : public rclcpp::Node
{
public:
  MultiRobotTfNode() : Node("multi_robot_tf_node")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // robot1: world -> robot1/base_link，平移 (1.0, 0.0, 0.0)
    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = this->now();
    t1.header.frame_id = "world";
    t1.child_frame_id = "robot1/base_link";
    t1.transform.translation.x = 1.0;
    t1.transform.translation.y = 0.0;
    t1.transform.translation.z = 0.0;
    t1.transform.rotation.w = 1.0;
    transforms.push_back(t1);

    // robot1: robot1/base_link -> robot1/sensor，平移 (0.0, 0.0, 0.5)
    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = this->now();
    t2.header.frame_id = "robot1/base_link";
    t2.child_frame_id = "robot1/sensor";
    t2.transform.translation.x = 0.0;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = 0.5;
    t2.transform.rotation.w = 1.0;
    transforms.push_back(t2);

    // robot2: world -> robot2/base_link，平移 (-1.0, 0.0, 0.0)
    geometry_msgs::msg::TransformStamped t3;
    t3.header.stamp = this->now();
    t3.header.frame_id = "world";
    t3.child_frame_id = "robot2/base_link";
    t3.transform.translation.x = -1.0;
    t3.transform.translation.y = 0.0;
    t3.transform.translation.z = 0.0;
    t3.transform.rotation.w = 1.0;
    transforms.push_back(t3);

    // robot2: robot2/base_link -> robot2/sensor，平移 (0.0, 0.0, 0.5)
    geometry_msgs::msg::TransformStamped t4;
    t4.header.stamp = this->now();
    t4.header.frame_id = "robot2/base_link";
    t4.child_frame_id = "robot2/sensor";
    t4.transform.translation.x = 0.0;
    t4.transform.translation.y = 0.0;
    t4.transform.translation.z = 0.5;
    t4.transform.rotation.w = 1.0;
    transforms.push_back(t4);

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
};

TEST_F(MultiRobotTfTest, CanCreateNode) {
  auto node = std::make_shared<MultiRobotTfNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(MultiRobotTfTest, PublishesFourTransforms) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  EXPECT_EQ(transforms.size(), 4u);
}

TEST_F(MultiRobotTfTest, Robot1BaseLinkFrameCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 1u);
  EXPECT_EQ(transforms[0].header.frame_id, "world");
  EXPECT_EQ(transforms[0].child_frame_id, "robot1/base_link");
}

TEST_F(MultiRobotTfTest, Robot1SensorFrameCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 2u);
  EXPECT_EQ(transforms[1].header.frame_id, "robot1/base_link");
  EXPECT_EQ(transforms[1].child_frame_id, "robot1/sensor");
}

TEST_F(MultiRobotTfTest, Robot2BaseLinkFrameCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 3u);
  EXPECT_EQ(transforms[2].header.frame_id, "world");
  EXPECT_EQ(transforms[2].child_frame_id, "robot2/base_link");
}

TEST_F(MultiRobotTfTest, Robot2SensorFrameCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 4u);
  EXPECT_EQ(transforms[3].header.frame_id, "robot2/base_link");
  EXPECT_EQ(transforms[3].child_frame_id, "robot2/sensor");
}

TEST_F(MultiRobotTfTest, Robot1BaseLinkTranslationCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 1u);
  EXPECT_DOUBLE_EQ(transforms[0].transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(transforms[0].transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transforms[0].transform.translation.z, 0.0);
}

TEST_F(MultiRobotTfTest, Robot1SensorTranslationCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 2u);
  EXPECT_DOUBLE_EQ(transforms[1].transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(transforms[1].transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transforms[1].transform.translation.z, 0.5);
}

TEST_F(MultiRobotTfTest, Robot2BaseLinkTranslationCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 3u);
  EXPECT_DOUBLE_EQ(transforms[2].transform.translation.x, -1.0);
  EXPECT_DOUBLE_EQ(transforms[2].transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transforms[2].transform.translation.z, 0.0);
}

TEST_F(MultiRobotTfTest, Robot2SensorTranslationCorrect) {
  auto node = std::make_shared<MultiRobotTfNode>();
  auto transforms = node->get_transforms();
  ASSERT_GE(transforms.size(), 4u);
  EXPECT_DOUBLE_EQ(transforms[3].transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(transforms[3].transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transforms[3].transform.translation.z, 0.5);
}

TEST_F(MultiRobotTfTest, CanLookupRobot1SensorFromWorld) {
  auto node = std::make_shared<MultiRobotTfNode>();

  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto transforms = node->get_transforms();
  for (const auto & t : transforms) {
    buffer->setTransform(t, "test_authority", true);
  }

  // 查询 world -> robot1/sensor 的累积变换
  // 期望: (1.0, 0.0, 0.0) + (0.0, 0.0, 0.5) = (1.0, 0.0, 0.5)
  try {
    auto result = buffer->lookupTransform("world", "robot1/sensor", tf2::TimePointZero);
    EXPECT_NEAR(result.transform.translation.x, 1.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.z, 0.5, 1e-6);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "无法查询 world -> robot1/sensor 变换: " << ex.what();
  }
}

TEST_F(MultiRobotTfTest, CanLookupRobot2SensorFromWorld) {
  auto node = std::make_shared<MultiRobotTfNode>();

  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto transforms = node->get_transforms();
  for (const auto & t : transforms) {
    buffer->setTransform(t, "test_authority", true);
  }

  // 查询 world -> robot2/sensor 的累积变换
  // 期望: (-1.0, 0.0, 0.0) + (0.0, 0.0, 0.5) = (-1.0, 0.0, 0.5)
  try {
    auto result = buffer->lookupTransform("world", "robot2/sensor", tf2::TimePointZero);
    EXPECT_NEAR(result.transform.translation.x, -1.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(result.transform.translation.z, 0.5, 1e-6);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "无法查询 world -> robot2/sensor 变换: " << ex.what();
  }
}
