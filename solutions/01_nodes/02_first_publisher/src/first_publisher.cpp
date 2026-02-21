#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class FirstPublisher : public rclcpp::Node
{
public:
  FirstPublisher() : Node("first_publisher"), count_(0)
  {
    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    timer_ = create_wall_timer(500ms, std::bind(&FirstPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2! Count: " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirstPublisher>());
  rclcpp::shutdown();
  return 0;
}
