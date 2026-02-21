// 参考答案 — service_client
// 修复：添加 wait_for_service、修正 async_send_request 参数、添加 spin_until_future_complete

#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("add_two_ints_client");
  auto client = node->create_client<ros2lings_interfaces::srv::AddTwoInts>("add_two_ints");

  // 等待服务可用
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<ros2lings_interfaces::srv::AddTwoInts::Request>();
  request->a = 5;
  request->b = 3;

  RCLCPP_INFO(node->get_logger(), "Sending request: %ld + %ld", request->a, request->b);

  // 正确传入 shared_ptr
  auto future = client->async_send_request(request);

  // 等待结果
  if (rclcpp::spin_until_future_complete(node, future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "Result: %ld", response->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
