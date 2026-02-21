// I AM NOT DONE
//
// 练习: first_service
// 模块: 02 - Services & Actions
// 难度: ★☆☆☆☆
//
// 学习目标:
//   理解 ROS2 Service Server 的创建方式和回调函数签名。
//
// 说明:
//   下面的代码尝试创建一个 AddTwoInts 服务服务器，但代码中有三个错误。
//   你需要找到并修复它们。
//
// 步骤:
//   1. 修复回调函数的签名（参数列表不完整）
//   2. 修复服务名称（不能包含空格）
//   3. 修复响应字段名（AddTwoInts 的响应字段是什么？）
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <memory>

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer() : Node("add_two_ints_server")
  {
    service_ = create_service<ros2lings_interfaces::srv::AddTwoInts>(
      "add two ints",
      std::bind(&AddTwoIntsServer::handle_add, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service server ready.");
  }

private:
  void handle_add(
    const std::shared_ptr<ros2lings_interfaces::srv::AddTwoInts::Request> request)
  {
    response->result = request->a + request->b;
    RCLCPP_INFO(get_logger(), "Request: %ld + %ld = %ld",
                request->a, request->b, response->result);
  }

  rclcpp::Service<ros2lings_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddTwoIntsServer>());
  rclcpp::shutdown();
  return 0;
}
