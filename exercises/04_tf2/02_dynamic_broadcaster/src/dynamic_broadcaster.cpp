// I AM NOT DONE
//
// 练习: dynamic_broadcaster
// 模块: 04 - TF2 Transforms
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 TransformBroadcaster（动态变换广播器）和 StaticTransformBroadcaster
//   的区别，掌握在定时器回调中持续发布动态坐标变换的方法。
//
// 说明:
//   下面的节点试图在定时器回调中持续发布一个随时间变化的动态坐标变换
//   （base_link -> sensor_link），但有 3 个错误需要你修复。
//
// 步骤:
//   1. 将 StaticTransformBroadcaster 改为 TransformBroadcaster — 动态变换需要动态广播器
//   2. 在定时器回调中用 this->now() 更新时间戳
//   3. 取消注释 sendTransform 调用
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class DynamicBroadcasterNode : public rclcpp::Node
{
public:
  DynamicBroadcasterNode() : Node("dynamic_broadcaster_node")
  {
    // BUG 1: 这里应该使用 TransformBroadcaster，而不是 StaticTransformBroadcaster
    // 动态变换需要持续发布，静态广播器只用于一次性的固定变换
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

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

    // BUG 2: 没有更新时间戳
    // 动态变换必须在每次回调中更新时间戳为当前时间
    // 正确做法: t.header.stamp = this->now();

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

    // BUG 3: sendTransform 调用被注释掉了，变换不会被发布
    // tf_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
