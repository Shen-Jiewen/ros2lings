#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class DynamicBroadcasterNode : public rclcpp::Node
{
public:
  DynamicBroadcasterNode() : Node("dynamic_broadcaster_node")
  {
    // 正确使用 TransformBroadcaster（非 Static）
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 创建定时器，每 100ms 发布一次变换
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DynamicBroadcasterNode::broadcast_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "动态变换广播器已启动");
  }

  // 获取广播器类型信息（供测试使用）
  std::string get_parent_frame() const { return "base_link"; }
  std::string get_child_frame() const { return "sensor_link"; }

private:
  void broadcast_timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;

    // 正确更新时间戳
    t.header.stamp = this->now();

    t.header.frame_id = "base_link";
    t.child_frame_id = "sensor_link";

    // 模拟传感器随时间做圆周运动
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
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
