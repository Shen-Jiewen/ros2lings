// I AM NOT DONE
//
// 练习: service_introspection
// 模块: 02 - Services & Actions
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 ROS2 服务的内省机制，学会用 ros2 CLI 工具查看和调用服务。
//
// 说明:
//   这是一个"探索"类练习。代码中有一些问答变量需要你根据
//   ROS2 的知识填写正确答案。
//
//   你可以运行以下命令来验证答案：
//   - ros2 service list              — 列出所有活跃服务
//   - ros2 service type <服务名>      — 查看服务的类型
//   - ros2 service call <服务名> ...  — 从命令行调用服务
//
// 步骤:
//   1. 根据 ROS2 知识填写每个 answer 变量的正确值
//   2. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <string>

// ===== 问答区域 =====

// 问题 1: 列出所有活跃服务的命令是？
// 选项: "ros2 service list" / "ros2 node list" / "ros2 topic list"
// TODO: 填写正确答案
std::string answer_list_services = "FILL_IN";

// 问题 2: Service 的通信模式是什么？
// 选项: "publish_subscribe" / "request_response" / "broadcast"
// TODO: 填写正确答案
std::string answer_service_pattern = "FILL_IN";

// 问题 3: 从命令行调用服务的命令是？
// 选项: "ros2 service call" / "ros2 service send" / "ros2 service invoke"
// TODO: 填写正确答案
std::string answer_call_command = "FILL_IN";

// 问题 4: 查看服务类型的命令是？
// 选项: "ros2 service type" / "ros2 service info" / "ros2 service show"
// TODO: 填写正确答案
std::string answer_type_command = "FILL_IN";

// 问题 5: 当服务端正常运行时，服务调用是否保证收到响应？
// 选项: "yes" / "no"
// TODO: 填写正确答案
std::string answer_guaranteed_response = "FILL_IN";

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
