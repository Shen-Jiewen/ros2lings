#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class QosMismatchNode : public rclcpp::Node
{
public:
  QosMismatchNode() : Node("qos_mismatch_node"), received_count_(0)
  {
    // 修复: 发布者和订阅者都使用 RELIABLE + TRANSIENT_LOCAL
    auto pub_qos = rclcpp::QoS(10);
    pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    publisher_ = this->create_publisher<std_msgs::msg::String>("qos_test_topic", pub_qos);

    auto sub_qos = rclcpp::QoS(10);
    sub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    sub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "qos_test_topic", sub_qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到消息: '%s'", msg->data.c_str());
        received_count_++;
      });

    timer_ = this->create_wall_timer(100ms, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "QoS 测试消息 #" + std::to_string(publish_count_++);
      publisher_->publish(msg);
    });
  }

  int get_received_count() const { return received_count_; }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  int received_count_ = 0;
  int publish_count_ = 0;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QosMismatchNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
