// 参考答案 — first_service
// 修复：回调签名、服务名称、响应字段名

#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <memory>

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer() : Node("add_two_ints_server")
  {
    service_ = create_service<ros2lings_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddTwoIntsServer::handle_add, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service server ready.");
  }

private:
  void handle_add(
    const std::shared_ptr<ros2lings_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<ros2lings_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "Request: %ld + %ld = %ld",
                request->a, request->b, response->sum);
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
