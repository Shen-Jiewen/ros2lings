#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TfTimeTravelNode : public rclcpp::Node
{
public:
  TfTimeTravelNode() : Node("tf_time_travel_node")
  {
    // 创建 Buffer 和 TransformListener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
#ifndef ROS2LINGS_TEST
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
#endif

    // 创建定时器，每 500ms 查询一次变换
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&TfTimeTravelNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "TF 时间旅行节点已启动");
  }

  // 访问函数（供测试使用）
  std::shared_ptr<tf2_ros::Buffer> get_buffer() const { return tf_buffer_; }

#ifdef ROS2LINGS_TEST
  // Test-only: track whether timer_callback successfully looked up a transform
  bool get_lookup_succeeded() const { return lookup_succeeded_; }
  int get_callback_count() const { return callback_count_; }
#endif

private:
  void timer_callback()
  {
    // 正确：设置合理的超时时间（1 秒）
    auto timeout = rclcpp::Duration::from_seconds(1.0);

    try {
      geometry_msgs::msg::TransformStamped t;
      // 正确：使用 tf2::TimePointZero 获取最新可用变换
      t = tf_buffer_->lookupTransform(
        "world",
        "robot",
        tf2::TimePointZero,
        timeout.to_chrono<std::chrono::nanoseconds>());

      RCLCPP_INFO(this->get_logger(),
        "变换 world -> robot: [%.2f, %.2f, %.2f]",
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z);
#ifdef ROS2LINGS_TEST
      lookup_succeeded_ = true;
#endif

    // 正确：捕获 tf2::TransformException
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "查询失败: %s", ex.what());
    }
#ifdef ROS2LINGS_TEST
    callback_count_++;
#endif
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
#ifdef ROS2LINGS_TEST
  bool lookup_succeeded_ = false;
  int callback_count_ = 0;
#endif
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfTimeTravelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
