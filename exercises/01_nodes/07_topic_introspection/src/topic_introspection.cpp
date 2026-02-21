// I AM NOT DONE
//
// 练习: topic_introspection
// 模块: 01 - Nodes & Topics
// 难度: ★☆☆☆☆
//
// 学习目标:
//   理解 ROS2 话题的内省机制，学会用 ros2 CLI 工具查看话题信息。
//
// 说明:
//   这是一个"探索"类练习。代码中有一些问答变量需要你根据
//   ROS2 的知识填写正确答案。
//
//   你可以运行以下命令来验证答案：
//   - ros2 topic list          — 列出所有活跃话题
//   - ros2 topic info <话题名>  — 查看话题的发布者/订阅者数量
//   - ros2 topic type <话题名>  — 查看话题的消息类型
//   - ros2 topic echo <话题名>  — 实时查看话题消息
//
// 步骤:
//   1. 根据 ROS2 知识填写每个 answer 变量的正确值
//   2. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

// ===== 问答区域 =====

// 问题 1: ros2 topic list 命令用于做什么？
// 选项: "list_nodes" / "list_topics" / "list_services"
// TODO: 填写正确答案
std::string answer_topic_list_purpose = "FILL_IN";

// 问题 2: 查看话题消息类型的命令是？
// 选项: "ros2 topic type" / "ros2 topic info" / "ros2 topic echo"
// TODO: 填写正确答案
std::string answer_type_command = "FILL_IN";

// 问题 3: std_msgs/msg/String 消息有一个字段，该字段名称是？
// 选项: "text" / "data" / "value"
// TODO: 填写正确答案
std::string answer_string_field = "FILL_IN";

// 问题 4: 哪个命令可以实时查看话题上的消息内容？
// 选项: "ros2 topic echo" / "ros2 topic info" / "ros2 topic pub"
// TODO: 填写正确答案
std::string answer_echo_command = "FILL_IN";

// 问题 5: 当没有发布者时，话题是否仍然存在于系统中？
// 选项: "yes" / "no"
// 提示: 只有有活跃的发布者或订阅者时，话题才会出现在 topic list 中
// TODO: 填写正确答案
std::string answer_topic_exists_without_pub = "FILL_IN";

// ===== 结束 =====

#ifndef TESTING_BUILD
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("topic_introspection_node");

  RCLCPP_INFO(node->get_logger(), "话题内省练习节点已启动");
  RCLCPP_INFO(node->get_logger(), "answer_topic_list_purpose = %s", answer_topic_list_purpose.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_type_command = %s", answer_type_command.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_string_field = %s", answer_string_field.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_echo_command = %s", answer_echo_command.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_topic_exists_without_pub = %s", answer_topic_exists_without_pub.c_str());

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
#endif
