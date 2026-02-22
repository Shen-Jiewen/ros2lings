// I AM NOT DONE
//
// 练习: frame_chain
// 模块: 04 - TF2 Transforms
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 TF2 中多帧链（frame chain）的概念，掌握如何用
//   StaticTransformBroadcaster 一次发布多个静态变换，构建一条完整的帧链。
//
// 说明:
//   下面的节点需要发布 3 个静态变换来构建帧链:
//     map -> odom -> base_link -> sensor_link
//   但关键部分需要你来补全。
//
// 步骤:
//   1. 创建 StaticTransformBroadcaster（TODO 1）
//   2. 创建第一个 TransformStamped: map -> odom，平移 (1.0, 0.0, 0.0)（TODO 2）
//   3. 创建第二个和第三个 TransformStamped（TODO 3）
//   4. 使用 sendTransform 发布所有变换（TODO 4）
//   5. 完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>

class FrameChainNode : public rclcpp::Node
{
public:
  FrameChainNode() : Node("frame_chain_node")
  {
    // TODO 1: 创建 StaticTransformBroadcaster
    // 提示: 使用 std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)
    tf_broadcaster_ = nullptr;

    // 存放所有变换
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // TODO 2: 创建第一个变换 map -> odom，平移 (1.0, 0.0, 0.0)
    // 提示:
    //   geometry_msgs::msg::TransformStamped t1;
    //   t1.header.stamp = this->now();
    //   t1.header.frame_id = "map";
    //   t1.child_frame_id = "odom";
    //   t1.transform.translation.x = 1.0;
    //   t1.transform.translation.y = 0.0;
    //   t1.transform.translation.z = 0.0;
    //   t1.transform.rotation.w = 1.0;
    //   transforms.push_back(t1);

    // TODO 3: 创建第二个变换 odom -> base_link，平移 (0.5, 0.0, 0.0)
    //         创建第三个变换 base_link -> sensor_link，平移 (0.0, 0.0, 0.3)
    // 提示: 参照 TODO 2 的模式，分别设置 frame_id、child_frame_id 和 translation

    // TODO 4: 使用 sendTransform 发布所有变换
    // 提示: tf_broadcaster_->sendTransform(transforms);

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
