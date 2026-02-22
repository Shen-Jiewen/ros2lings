// I AM NOT DONE
//
// 练习: static_broadcaster
// 模块: 04 - TF2 Transforms
// 难度: ★☆☆☆☆
//
// 学习目标:
//   理解 StaticTransformBroadcaster 的使用方法，掌握如何发布一个静态坐标变换
//   （从父帧到子帧的固定空间关系）。
//
// 说明:
//   下面的节点试图发布一个从 "world" 到 "base_link" 的静态坐标变换，
//   但有 3 个错误需要你修复。
//
// 步骤:
//   1. 添加缺失的头文件 — 需要 include tf2_ros/static_transform_broadcaster.h
//   2. 修复 frame_id 和 child_frame_id 的赋值 — 它们被设置反了
//   3. 修复四元数的 w 分量 — 单位四元数的 w 应为 1.0
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// BUG 1: 缺少 tf2_ros/static_transform_broadcaster.h 的头文件
// #include <tf2_ros/static_transform_broadcaster.h>

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

    // BUG 2: frame_id 和 child_frame_id 设置反了
    // 正确的关系应该是: world (父帧) -> base_link (子帧)
    t.header.frame_id = "base_link";     // 应该是 "world"
    t.child_frame_id = "world";           // 应该是 "base_link"

    // 设置平移 (x=1.0, y=2.0, z=0.5)
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.5;

    // BUG 3: 四元数 w 分量应为 1.0（单位四元数表示无旋转）
    // w=0.0 会导致无效的旋转
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.0;  // 应该是 1.0

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
