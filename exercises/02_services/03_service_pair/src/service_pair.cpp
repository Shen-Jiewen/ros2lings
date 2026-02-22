// I AM NOT DONE
//
// 练习: service_pair
// 模块: 02 - Services & Actions
// 难度: ★★☆☆☆
//
// 学习目标:
//   将 Service Server 和 Client 结合起来，实现完整的请求/应答流程。
//
// 说明:
//   你需要补全下面的代码，实现一个完整的 Service Server + Client 调用流程。
//   框架代码已经给出，请根据 TODO 提示完成实现。
//
// 步骤:
//   1. 在 ServerNode 构造函数中创建服务
//   2. 实现 handle_add 回调函数
//   3. 在 main 函数中创建客户端、发送请求并等待结果
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

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
    // TODO: 使用 create_service 创建 AddTwoInts 服务
    // 服务名: "add_two_ints"
    // 回调: std::bind(&ServerNode::handle_add, this, _1, _2)

    RCLCPP_INFO(get_logger(), "Server ready.");
  }

private:
  void handle_add(
    const std::shared_ptr<AddTwoInts::Request> request,
    std::shared_ptr<AddTwoInts::Response> response)
  {
    // TODO: 计算 response->sum = request->a + request->b
    // TODO: 打印日志

  }

  rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 创建服务器节点
  auto server_node = std::make_shared<ServerNode>();

  // 创建客户端节点
  auto client_node = std::make_shared<rclcpp::Node>("client_node");

  // TODO: 创建客户端
  // auto client = client_node->create_client<AddTwoInts>("add_two_ints");

  // TODO: 等待服务可用
  // if (!client->wait_for_service(5s)) { ... }

  // 构造请求
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 42;
  request->b = 58;

  RCLCPP_INFO(client_node->get_logger(), "Sending: %ld + %ld", request->a, request->b);

  // TODO: 发送异步请求
  // auto future = client->async_send_request(request);

  // TODO: 使用 executor 同时 spin 两个节点，等待结果
  // 提示: SingleThreadedExecutor, add_node, spin_until_future_complete
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(server_node);
  // executor.add_node(client_node);
  // executor.spin_until_future_complete(future);

  // TODO: 获取并打印结果
  // auto response = future.get();
  // RCLCPP_INFO(client_node->get_logger(), "Result: %ld", response->sum);

  rclcpp::shutdown();
  return 0;
}
#endif
