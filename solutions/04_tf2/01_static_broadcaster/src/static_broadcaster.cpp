#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class StaticBroadcasterNode : public rclcpp::Node
{
public:
  StaticBroadcasterNode() : Node("static_broadcaster_node")
  {
    // 创建静态变换广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 构建变换消息
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();

    // 正确的帧关系: world (父帧) -> base_link (子帧)
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";

    // 设置平移 (x=1.0, y=2.0, z=0.5)
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.5;

    // 单位四元数表示无旋转
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // 发布静态变换
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(), "已发布静态变换: %s -> %s",
                t.header.frame_id.c_str(), t.child_frame_id.c_str());
  }

  // 获取变换信息的访问函数（供测试使用）
  std::string get_parent_frame() const { return parent_frame_; }
  std::string get_child_frame() const { return child_frame_; }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::string parent_frame_ = "world";
  std::string child_frame_ = "base_link";
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
