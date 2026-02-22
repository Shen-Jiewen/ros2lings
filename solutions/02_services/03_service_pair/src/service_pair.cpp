// 参考答案 — service_pair
// 实现完整的 Service Server + Client 调用流程

#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using AddTwoInts = ros2lings_interfaces::srv::AddTwoInts;

// ===== 服务服务器节点 =====
class ServerNode : public rclcpp::Node
{
public:
  ServerNode() : Node("server_node")
  {
    service_ = create_service<AddTwoInts>(
      "add_two_ints",
      std::bind(&ServerNode::handle_add, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Server ready.");
  }

private:
  void handle_add(
    const std::shared_ptr<AddTwoInts::Request> request,
    std::shared_ptr<AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "%ld + %ld = %ld",
                request->a, request->b, response->sum);
  }

  rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 创建服务器节点
  auto server_node = std::make_shared<ServerNode>();

  // 创建客户端节点
  auto client_node = std::make_shared<rclcpp::Node>("client_node");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");

  // 等待服务可用
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(client_node->get_logger(), "Service not available");
    rclcpp::shutdown();
    return 1;
  }

  // 构造请求
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 42;
  request->b = 58;

  RCLCPP_INFO(client_node->get_logger(), "Sending: %ld + %ld", request->a, request->b);

  // 发送异步请求
  auto future = client->async_send_request(request);

  // 使用 executor 同时 spin 两个节点来等待结果
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  if (executor.spin_until_future_complete(future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(client_node->get_logger(), "Result: %ld", response->sum);
  } else {
    RCLCPP_ERROR(client_node->get_logger(), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
