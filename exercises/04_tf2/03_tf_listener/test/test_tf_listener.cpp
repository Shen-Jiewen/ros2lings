#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

// 在测试文件中定义正确的节点类
class TfListenerNode : public rclcpp::Node
{
public:
  TfListenerNode() : Node("tf_listener_node")
  {
    // 正确：传入节点时钟
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TfListenerNode::timer_callback, this));
  }

  std::string get_target_frame() const { return target_frame_; }
  std::string get_source_frame() const { return source_frame_; }
  std::shared_ptr<tf2_ros::Buffer> get_buffer() const { return tf_buffer_; }

private:
  void timer_callback()
  {
    // 正确的参数顺序和异常处理
    try {
      geometry_msgs::msg::TransformStamped t;
      t = tf_buffer_->lookupTransform(
        target_frame_,   // 目标帧: "base_link"
        source_frame_,   // 源帧: "sensor_link"
        tf2::TimePointZero);

      RCLCPP_INFO(this->get_logger(),
        "变换 base_link -> sensor_link: [%.2f, %.2f, %.2f]",
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string target_frame_ = "base_link";
  std::string source_frame_ = "sensor_link";
};

class TfListenerTest : public ::testing::Test {
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

TEST_F(TfListenerTest, CanCreateNode) {
  auto node = std::make_shared<TfListenerNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(TfListenerTest, BufferExists) {
  auto node = std::make_shared<TfListenerNode>();
  ASSERT_NE(node->get_buffer(), nullptr);
}

TEST_F(TfListenerTest, TargetFrameIsBaseLink) {
  auto node = std::make_shared<TfListenerNode>();
  EXPECT_EQ(node->get_target_frame(), "base_link");
}

TEST_F(TfListenerTest, SourceFrameIsSensorLink) {
  auto node = std::make_shared<TfListenerNode>();
  EXPECT_EQ(node->get_source_frame(), "sensor_link");
}

TEST_F(TfListenerTest, LookupTransformHandlesException) {
  auto node = std::make_shared<TfListenerNode>();
  // lookupTransform 在没有变换数据时应该抛出异常
  // 正确的实现会用 try-catch 处理，不会导致节点崩溃
  // 这里通过 spin_some 触发定时器回调来验证节点不会崩溃
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1500)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  // 如果执行到这里没有崩溃，说明异常处理是正确的
  SUCCEED();
}

TEST_F(TfListenerTest, LookupTransformFrameOrder) {
  auto node = std::make_shared<TfListenerNode>();
  auto buffer = node->get_buffer();

  // 手动向 buffer 中添加一个变换，用于验证查询顺序
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "sensor_link";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  // 验证正确的查询顺序: lookupTransform(target, source, time)
  // 查询 "sensor_link 在 base_link 坐标系中的位姿"
  try {
    auto result = buffer->lookupTransform("base_link", "sensor_link", tf2::TimePointZero);
    EXPECT_EQ(result.header.frame_id, "base_link");
    EXPECT_EQ(result.child_frame_id, "sensor_link");
  } catch (const tf2::TransformException & ex) {
    FAIL() << "lookupTransform 不应抛出异常: " << ex.what();
  }
}
