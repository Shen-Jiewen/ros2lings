#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

// ===== 问答区域 =====

std::string answer_topic_list_purpose = "list_topics";
std::string answer_type_command = "ros2 topic type";
std::string answer_string_field = "data";
std::string answer_echo_command = "ros2 topic echo";
std::string answer_topic_exists_without_pub = "no";

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
