// 参考答案 — service_introspection
// 服务内省练习: 所有答案已填写

#include <rclcpp/rclcpp.hpp>
#include <string>

// ===== 问答区域 =====

std::string answer_list_services = "ros2 service list";
std::string answer_service_pattern = "request_response";
std::string answer_call_command = "ros2 service call";
std::string answer_type_command = "ros2 service type";
std::string answer_guaranteed_response = "yes";

// ===== 结束 =====

#ifndef TESTING_BUILD
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("service_introspection_node");

  RCLCPP_INFO(node->get_logger(), "服务内省练习节点已启动");
  RCLCPP_INFO(node->get_logger(), "answer_list_services = %s", answer_list_services.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_service_pattern = %s", answer_service_pattern.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_call_command = %s", answer_call_command.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_type_command = %s", answer_type_command.c_str());
  RCLCPP_INFO(node->get_logger(), "answer_guaranteed_response = %s", answer_guaranteed_response.c_str());

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
#endif
