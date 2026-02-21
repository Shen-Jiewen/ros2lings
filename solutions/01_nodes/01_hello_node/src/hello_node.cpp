#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("hello_node");

  RCLCPP_INFO(node->get_logger(), "Hello, ROS2! I am a node.");

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
