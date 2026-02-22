// I AM NOT DONE
//
// 练习: service_client
// 模块: 02 - Services & Actions
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 ROS2 Service Client 的异步调用流程，包括等待服务、
//   发送请求和等待结果。
//
// 说明:
//   下面的代码尝试创建一个 AddTwoInts 服务客户端，但代码中有三个错误。
//   你需要找到并修复它们。
//
// 步骤:
//   1. 在发送请求前等待服务可用
//   2. 修复 async_send_request 的参数（提示：不需要 .get()）
//   3. 在获取结果前等待 future 完成
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("add_two_ints_client");
  auto client = node->create_client<ros2lings_interfaces::srv::AddTwoInts>("add_two_ints");

  // BUG 1: 没有等待服务可用就直接发送请求

  auto request = std::make_shared<ros2lings_interfaces::srv::AddTwoInts::Request>();
  request->a = 5;
  request->b = 3;

  RCLCPP_INFO(node->get_logger(), "Sending request: %ld + %ld", request->a, request->b);

  // BUG 2: async_send_request 需要 shared_ptr，不是裸指针
  auto future = client->async_send_request(request.get());

  // BUG 3: 没有等待 future 完成就直接获取结果
  auto response = future.get();
  RCLCPP_INFO(node->get_logger(), "Result: %ld", response->sum);

  rclcpp::shutdown();
  return 0;
}
#endif
