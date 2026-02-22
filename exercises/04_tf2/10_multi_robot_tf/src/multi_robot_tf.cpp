// I AM NOT DONE
//
// 练习: multi_robot_tf
// 模块: 04 - TF2 Transforms
// 难度: ★★★☆☆
//
// 学习目标:
//   理解多机器人系统中 TF 树的组织方式，掌握使用帧名前缀进行命名空间隔离。
//
// 说明:
//   在多机器人系统中，每个机器人都有自己的坐标帧（如 base_link、sensor）。
//   为了避免帧名冲突，通常使用机器人名称作为前缀（如 robot1/base_link）。
//   本练习需要你发布两个机器人的静态 TF 树：
//
//   robot1: world -> robot1/base_link (平移 1.0, 0.0, 0.0)
//           robot1/base_link -> robot1/sensor (平移 0.0, 0.0, 0.5)
//
//   robot2: world -> robot2/base_link (平移 -1.0, 0.0, 0.0)
//           robot2/base_link -> robot2/sensor (平移 0.0, 0.0, 0.5)
//
// 步骤:
//   1. 创建 StaticTransformBroadcaster（TODO 1）
//   2. 创建 robot1 的两个变换（TODO 2）
//   3. 创建 robot2 的两个变换（TODO 3）
//   4. 发布所有变换（TODO 4）
//   5. 完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>

class MultiRobotTfNode : public rclcpp::Node
{
public:
  MultiRobotTfNode() : Node("multi_robot_tf_node")
  {
    // TODO 1: 创建 StaticTransformBroadcaster
    // 提示: tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 存放所有变换
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // TODO 2: 创建 robot1 的两个变换
    // 变换 1: world -> robot1/base_link，平移 (1.0, 0.0, 0.0)
    // 变换 2: robot1/base_link -> robot1/sensor，平移 (0.0, 0.0, 0.5)
    // 提示:
    //   geometry_msgs::msg::TransformStamped t;
    //   t.header.stamp = this->now();
    //   t.header.frame_id = "world";
    //   t.child_frame_id = "robot1/base_link";
    //   t.transform.translation.x = 1.0;
    //   t.transform.rotation.w = 1.0;
    //   transforms.push_back(t);
    //   ... 类似地创建第二个变换 ...

    // TODO 3: 创建 robot2 的两个变换（使用 robot2/ 前缀）
    // 变换 3: world -> robot2/base_link，平移 (-1.0, 0.0, 0.0)
    // 变换 4: robot2/base_link -> robot2/sensor，平移 (0.0, 0.0, 0.5)
    // 提示: 参照 TODO 2 的模式，将 robot1 替换为 robot2，调整平移值

    // TODO 4: 使用 sendTransform 发布所有变换
    // 提示: tf_broadcaster_->sendTransform(transforms);

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
