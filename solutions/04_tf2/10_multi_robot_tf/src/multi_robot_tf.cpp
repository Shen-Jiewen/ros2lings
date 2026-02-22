#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>

class MultiRobotTfNode : public rclcpp::Node
{
public:
  MultiRobotTfNode() : Node("multi_robot_tf_node")
  {
    // 创建静态变换广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 存放所有变换
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

    // 发布所有变换
    tf_broadcaster_->sendTransform(transforms);

    RCLCPP_INFO(this->get_logger(),
      "已发布多机器人 TF 树: world -> robot1/*, world -> robot2/*");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiRobotTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
