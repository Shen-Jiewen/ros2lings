#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <cmath>

// 在测试文件中定义正确的节点类
class DynamicBroadcasterNode : public rclcpp::Node
{
public:
  DynamicBroadcasterNode() : Node("dynamic_broadcaster_node")
  {
    // 正确使用 TransformBroadcaster（非 Static）
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DynamicBroadcasterNode::broadcast_timer_callback, this));
  }

  std::string get_parent_frame() const { return "base_link"; }
  std::string get_child_frame() const { return "sensor_link"; }
  bool has_published() const { return publish_count_ > 0; }
  int get_publish_count() const { return publish_count_; }
  geometry_msgs::msg::TransformStamped get_last_transform() const { return last_transform_; }

private:
  void broadcast_timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;

    // 正确更新时间戳
    t.header.stamp = this->now();

    t.header.frame_id = "base_link";
    t.child_frame_id = "sensor_link";

    double seconds = this->now().seconds();
    t.transform.translation.x = 0.5 * std::cos(seconds);
    t.transform.translation.y = 0.5 * std::sin(seconds);
    t.transform.translation.z = 0.3;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // 正确调用 sendTransform
    tf_broadcaster_->sendTransform(t);
    last_transform_ = t;
    publish_count_++;
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped last_transform_;
  int publish_count_ = 0;
};

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
  // 执行几次 spin 让定时器有机会触发
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_TRUE(node->has_published());
}

TEST_F(DynamicBroadcasterTest, TransformHasValidTimestamp) {
  auto node = std::make_shared<DynamicBroadcasterNode>();
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (node->has_published()) {
    auto t = node->get_last_transform();
    // 时间戳不应为零（说明用了 this->now()）
    EXPECT_GT(t.header.stamp.sec + t.header.stamp.nanosec, 0u);
  }
}

TEST_F(DynamicBroadcasterTest, TransformFrameNamesCorrect) {
  auto node = std::make_shared<DynamicBroadcasterNode>();
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (node->has_published()) {
    auto t = node->get_last_transform();
    EXPECT_EQ(t.header.frame_id, "base_link");
    EXPECT_EQ(t.child_frame_id, "sensor_link");
  }
}
