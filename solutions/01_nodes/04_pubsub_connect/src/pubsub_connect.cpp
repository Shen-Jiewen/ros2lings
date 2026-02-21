#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PubSubConnect : public rclcpp::Node
{
public:
  PubSubConnect() : Node("pubsub_connect"), count_(0), received_count_(0)
  {
    publisher_ = create_publisher<std_msgs::msg::String>("ping", 10);

    subscription_ = create_subscription<std_msgs::msg::String>(
      "ping", 10,
      std::bind(&PubSubConnect::sub_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&PubSubConnect::timer_callback, this));
  }

  size_t get_received_count() const { return received_count_; }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Ping #" + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void sub_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received: '%s'", msg->data.c_str());
    received_count_++;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  size_t received_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSubConnect>());
  rclcpp::shutdown();
  return 0;
}
