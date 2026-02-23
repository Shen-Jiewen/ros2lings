#include <rclcpp/rclcpp.hpp>
#include <string>

// ===== 问答区域 =====

std::string answer_node_name = "my_node";
std::string answer_default_namespace = "/";
std::string answer_init_purpose = "init_context";
std::string answer_spin_purpose = "event_loop";
std::string answer_default_topics = "rosout";

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
