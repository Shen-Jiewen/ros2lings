// I AM NOT DONE
//
// 练习: namespace_remap
// 模块: 01 - Nodes & Topics
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 ROS2 节点命名空间和话题重映射的机制。
//
// 说明:
//   下面的代码中存在两个问题：
//   1. 节点的命名空间设置不正确
//   2. 话题名没有被正确重映射
//
//   节点应该在 "/robot1" 命名空间下运行，
//   发布者的话题应该从 "raw_data" 重映射到 "sensor_data"。
//
// 背景知识:
//   - 命名空间用于隔离节点，避免名称冲突
//   - 在 NodeOptions 中设置命名空间
//   - 话题重映射允许在不修改代码的情况下改变话题名称
//
// 步骤:
//   1. 修复节点的命名空间设置
//   2. 修复话题重映射，使 "raw_data" 被映射到 "sensor_data"
//   3. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class NamespaceRemapNode : public rclcpp::Node
{
public:
  NamespaceRemapNode()
  // BUG 1: 命名空间应该是 "robot1"，但这里设置错误
  // 提示: Node 构造函数的第二个参数是命名空间
  : Node("data_publisher", "wrong_namespace")
  {
    // BUG 2: 话题名应该通过重映射从 "raw_data" 变成 "sensor_data"
    // 但这里直接使用了错误的话题名
    publisher_ = this->create_publisher<std_msgs::msg::String>("raw_data", 10);

    timer_ = this->create_wall_timer(200ms, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "传感器数据 #" + std::to_string(count_++);
      publisher_->publish(msg);
    });
  }

  std::string get_topic_name() const {
    return publisher_->get_topic_name();
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NamespaceRemapNode>();

  RCLCPP_INFO(node->get_logger(), "节点名: %s", node->get_name());
  RCLCPP_INFO(node->get_logger(), "命名空间: %s", node->get_namespace());
  RCLCPP_INFO(node->get_logger(), "话题: %s", node->get_topic_name().c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
