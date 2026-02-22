#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PublisherComponent : public rclcpp::Node
{
public:
  explicit PublisherComponent(const rclcpp::NodeOptions & options)
  : Node("publisher_component", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("composition_topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&PublisherComponent::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "PublisherComponent started");
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Composed message #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

RCLCPP_COMPONENTS_REGISTER_NODE(PublisherComponent)
