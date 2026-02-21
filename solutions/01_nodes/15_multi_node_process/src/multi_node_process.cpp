#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ProducerNode : public rclcpp::Node
{
public:
  ProducerNode() : Node("producer_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("internal_topic", 10);
    timer_ = this->create_wall_timer(200ms, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "生产消息 #" + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "发布: '%s'", msg.data.c_str());
      publisher_->publish(msg);
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

class ConsumerNode : public rclcpp::Node
{
public:
  ConsumerNode() : Node("consumer_node"), received_count_(0)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "internal_topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到: '%s'", msg->data.c_str());
        received_count_++;
      });
  }

  int get_received_count() const { return received_count_; }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int received_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto producer = std::make_shared<ProducerNode>();
  auto consumer = std::make_shared<ConsumerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(producer);
  executor.add_node(consumer);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
