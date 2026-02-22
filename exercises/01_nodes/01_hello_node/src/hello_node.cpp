// I AM NOT DONE
//
// 练习: hello_node
// 模块: 01 - Nodes & Topics
// 难度: ★☆☆☆☆
//
// 学习目标:
//   理解 ROS2 节点的创建和 rclcpp::init/shutdown 生命周期。
//
// 说明:
//   下面的代码尝试创建一个最简单的 ROS2 节点并让它打印一条消息。
//   但代码中有几个错误需要你修复。
//
// 步骤:
//   1. 修复 rclcpp::init() 的参数
//   2. 修复节点的创建方式（提示：需要智能指针）
//   3. 确保节点能正确 spin 一次
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  // TODO: rclcpp::init() 需要 argc 和 argv 来初始化 ROS2 上下文
  rclcpp::init();

  // TODO: 节点需要用 std::make_shared 创建，而不是直接构造
  auto node = rclcpp::Node("hello_node");

  RCLCPP_INFO(node->get_logger(), "Hello, ROS2! I am a node.");

  // TODO: 让节点至少运行一次（提示：spin_some 或 spin_once）
  rclcpp::shutdown();
  return 0;
}
#endif
