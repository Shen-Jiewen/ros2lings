#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class FirstSubscriber : public rclcpp::Node
{
public:
  FirstSubscriber() : Node("first_subscriber")
  {
    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      std::bind(&FirstSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
    last_msg_ = msg->data;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string last_msg_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirstSubscriber>());
  rclcpp::shutdown();
  return 0;
}
#endif
