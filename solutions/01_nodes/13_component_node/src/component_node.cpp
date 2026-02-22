#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TalkerComponent : public rclcpp::Node
{
public:
  explicit TalkerComponent(const rclcpp::NodeOptions & options)
  : Node("talker_component", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("component_chatter", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&TalkerComponent::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "来自组件的消息 #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "发布: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

#ifndef ROS2LINGS_TEST
RCLCPP_COMPONENTS_REGISTER_NODE(TalkerComponent)
#endif
