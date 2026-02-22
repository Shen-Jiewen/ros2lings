#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberComponent : public rclcpp::Node
{
public:
  explicit SubscriberComponent(const rclcpp::NodeOptions & options)
  : Node("subscriber_component", options)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "composition_topic", 10,
      std::bind(&SubscriberComponent::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "SubscriberComponent started");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(SubscriberComponent)
