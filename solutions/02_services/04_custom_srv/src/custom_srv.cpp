// 参考答案 — custom_srv
// 实现自定义 ComputeArea 服务的服务器和客户端

#include <rclcpp/rclcpp.hpp>
#include <ros2lings_19_custom_srv/srv/compute_area.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using ComputeArea = ros2lings_19_custom_srv::srv::ComputeArea;

class AreaServer : public rclcpp::Node
{
public:
  AreaServer() : Node("area_server")
  {
    service_ = create_service<ComputeArea>(
      "compute_area",
      std::bind(&AreaServer::handle_compute, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Area server ready.");
  }

private:
  void handle_compute(
    const std::shared_ptr<ComputeArea::Request> request,
    std::shared_ptr<ComputeArea::Response> response)
  {
    response->area = request->width * request->height;
    RCLCPP_INFO(get_logger(), "%.2f x %.2f = %.2f",
                request->width, request->height, response->area);
  }

  rclcpp::Service<ComputeArea>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto server_node = std::make_shared<AreaServer>();
  auto client_node = std::make_shared<rclcpp::Node>("area_client");

  auto client = client_node->create_client<ComputeArea>("compute_area");

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(client_node->get_logger(), "Service not available");
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<ComputeArea::Request>();
  request->width = 3.5;
  request->height = 4.2;

  RCLCPP_INFO(client_node->get_logger(), "Sending: %.2f x %.2f",
              request->width, request->height);

  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(server_node, future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(client_node->get_logger(), "Area: %.2f", response->area);
  } else {
    RCLCPP_ERROR(client_node->get_logger(), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
