#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>

class FrameChainNode : public rclcpp::Node
{
public:
  FrameChainNode() : Node("frame_chain_node")
  {
    // 创建静态变换广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 存放所有变换
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

    // 发布所有变换
    tf_broadcaster_->sendTransform(transforms);

    RCLCPP_INFO(this->get_logger(),
      "已发布帧链: map -> odom -> base_link -> sensor_link");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrameChainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
