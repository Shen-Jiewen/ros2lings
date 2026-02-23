// I AM NOT DONE
//
// 练习: custom_srv
// 模块: 02 - Services & Actions
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解如何使用自定义 .srv 文件定义的服务接口。
//
// 说明:
//   你需要补全下面的代码，使用自定义的 ComputeArea 服务。
//   该服务接收 width 和 height，返回 area。
//   请根据 TODO 提示完成实现。
//
// 步骤:
//   1. 添加正确的头文件 include（提示：CamelCase 转 snake_case）
//   2. 添加类型别名
//   3. 实现服务回调中的面积计算
//   4. 完成 main 函数中的客户端逻辑
//   5. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
// TODO: 添加自定义服务头文件
// 提示: 包名是 ros2lings_19_custom_srv，服务名是 ComputeArea
// #include <ros2lings_19_custom_srv/srv/compute_area.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
// TODO: 添加类型别名
// using ComputeArea = ros2lings_19_custom_srv::srv::ComputeArea;

class AreaServer : public rclcpp::Node
{
public:
  AreaServer() : Node("area_server")
  {
    // TODO: 创建 ComputeArea 服务（取消注释并修复）
    // service_ = create_service<ComputeArea>(
    //   "compute_area",
    //   std::bind(&AreaServer::handle_compute, this,
    //             std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Area server ready.");
  }

private:
  void handle_compute(
    const std::shared_ptr<ComputeArea::Request> request,
    std::shared_ptr<ComputeArea::Response> response)
  {
    // TODO: 计算面积 = width * height
    // response->area = request->width * request->height;
    RCLCPP_INFO(get_logger(), "%.2f x %.2f = %.2f",
                request->width, request->height, response->area);
  }

  // TODO: 声明服务成员变量
  // rclcpp::Service<ComputeArea>::SharedPtr service_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto server_node = std::make_shared<AreaServer>();
  auto client_node = std::make_shared<rclcpp::Node>("area_client");

  // TODO: 创建客户端
  // auto client = client_node->create_client<ComputeArea>("compute_area");

  // TODO: 等待服务可用
  // if (!client->wait_for_service(5s)) { ... }

  // TODO: 创建请求并设置 width 和 height
  // auto request = std::make_shared<ComputeArea::Request>();
  // request->width = 3.5;
  // request->height = 4.2;

  // TODO: 发送请求并使用 executor 等待结果
  // 提示: SingleThreadedExecutor, add_node(server_node), add_node(client_node)
  // auto future = client->async_send_request(request);
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(server_node);
  // executor.add_node(client_node);
  // executor.spin_until_future_complete(future);
  // auto response = future.get();
  // RCLCPP_INFO(client_node->get_logger(), "Area: %.2f", response->area);

  rclcpp::shutdown();
  return 0;
}
#endif
