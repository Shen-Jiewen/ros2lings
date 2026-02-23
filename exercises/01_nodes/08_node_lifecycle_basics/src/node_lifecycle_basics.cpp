// I AM NOT DONE
//
// 练习: node_lifecycle_basics
// 模块: 01 - Nodes & Topics
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 ROS2 Node 构造函数内部做了什么，了解节点的基本属性。
//
// 说明:
//   这是一个"探索"类练习。你需要根据对 ROS2 Node 内部机制的理解
//   填写正确的答案。
//
//   提示：创建一个节点后，试试以下方法：
//   - node->get_name()
//   - node->get_namespace()
//   - node->get_logger()
//   - node->get_clock()
//   - node->count_publishers("话题名")
//   - node->count_subscribers("话题名")
//
// 步骤:
//   1. 根据 ROS2 Node 的知识填写每个 answer 变量
//   2. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <string>

// ===== 问答区域 =====

// 问题 1: 创建节点 std::make_shared<rclcpp::Node>("my_node") 后，
//         node->get_name() 返回什么？
// TODO: 填写正确答案
std::string answer_node_name = "FILL_IN";

// 问题 2: 默认情况下，node->get_namespace() 返回什么？
// 选项: "" / "/" / "default"
// TODO: 填写正确答案
std::string answer_default_namespace = "FILL_IN";

// 问题 3: rclcpp::init() 的主要作用是什么？
// 选项: "create_node" / "init_context" / "start_spin"
// TODO: 填写正确答案
std::string answer_init_purpose = "FILL_IN";

// 问题 4: rclcpp::spin(node) 的作用是什么？
// 选项: "create_threads" / "event_loop" / "shutdown"
// TODO: 填写正确答案
std::string answer_spin_purpose = "FILL_IN";

// 问题 5: 一个刚创建的节点（没有创建任何发布者/订阅者），
//         node->get_topic_names_and_types() 会包含哪些话题？
// 选项: "none" / "rosout" / "all_system_topics"
// 提示: 每个节点默认都有一个用于日志的话题
// TODO: 填写正确答案
std::string answer_default_topics = "FILL_IN";

// ===== 结束 =====

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lifecycle_basics_node");

  RCLCPP_INFO(node->get_logger(), "节点生命周期基础练习已启动");
  RCLCPP_INFO(node->get_logger(), "节点名称: %s", node->get_name());
  RCLCPP_INFO(node->get_logger(), "节点命名空间: %s", node->get_namespace());

  RCLCPP_INFO(node->get_logger(), "answer_node_name = %s", answer_node_name.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_default_namespace = %s", answer_default_namespace.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_init_purpose = %s", answer_init_purpose.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_spin_purpose = %s", answer_spin_purpose.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_default_topics = %s", answer_default_topics.c_str());

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
#endif
