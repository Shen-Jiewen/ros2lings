#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TfListenerNode : public rclcpp::Node
{
public:
  TfListenerNode() : Node("tf_listener_node")
  {
    // 正确：传入节点时钟
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    // 创建 TransformListener，它会自动订阅 /tf 和 /tf_static
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建定时器，每 500ms 查询一次变换
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TfListenerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "TF 监听器已启动，等待变换数据...");
  }

  // 访问函数（供测试使用）
  std::string get_target_frame() const { return target_frame_; }
  std::string get_source_frame() const { return source_frame_; }
  std::shared_ptr<tf2_ros::Buffer> get_buffer() const { return tf_buffer_; }

private:
  void timer_callback()
  {
    // 正确的异常处理和参数顺序
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

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
