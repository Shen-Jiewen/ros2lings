#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class NamespaceRemapNode : public rclcpp::Node
{
public:
  NamespaceRemapNode()
  // 修复 1: 命名空间改为 "robot1"
  : Node("data_publisher", "robot1")
  {
    // 修复 2: 话题名改为 "sensor_data"（重映射后的名称）
    publisher_ = this->create_publisher<std_msgs::msg::String>("sensor_data", 10);

    timer_ = this->create_wall_timer(200ms, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "传感器数据 #" + std::to_string(count_++);
      publisher_->publish(msg);
    });
  }

  std::string get_topic_name() const {
    return publisher_->get_topic_name();
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NamespaceRemapNode>();

  RCLCPP_INFO(node->get_logger(), "节点名: %s", node->get_name());
  RCLCPP_INFO(node->get_logger(), "命名空间: %s", node->get_namespace());
  RCLCPP_INFO(node->get_logger(), "话题: %s", node->get_topic_name().c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
