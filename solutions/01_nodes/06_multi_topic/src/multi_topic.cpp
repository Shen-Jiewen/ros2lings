#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MultiTopicNode : public rclcpp::Node
{
public:
  MultiTopicNode() : Node("multi_topic_node"), count_(0)
  {
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);

    command_pub_ = create_publisher<std_msgs::msg::String>("command", 10);

    status_sub_ = create_subscription<std_msgs::msg::String>(
      "status", 10,
      std::bind(&MultiTopicNode::status_callback, this, std::placeholders::_1));

    command_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10,
      std::bind(&MultiTopicNode::command_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&MultiTopicNode::timer_callback, this));
  }

  std::string get_last_status() const { return last_status_; }
  std::string get_last_command() const { return last_command_; }

private:
  void timer_callback()
  {
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "Status: OK #" + std::to_string(count_);
    status_pub_->publish(status_msg);

    auto cmd_msg = std_msgs::msg::String();
    cmd_msg.data = "Command: RUN #" + std::to_string(count_);
    command_pub_->publish(cmd_msg);

    count_++;
  }

  void status_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Status received: '%s'", msg->data.c_str());
    last_status_ = msg->data;
  }

  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Command received: '%s'", msg->data.c_str());
    last_command_ = msg->data;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::string last_status_;
  std::string last_command_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiTopicNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
