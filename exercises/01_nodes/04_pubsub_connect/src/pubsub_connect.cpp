// I AM NOT DONE
//
// 练习: pubsub_connect
// 模块: 01 - Nodes & Topics
// 难度: ★★☆☆☆
//
// 学习目标:
//   在同一个节点中同时实现发布者和订阅者，理解完整的 pub/sub 通信系统。
//
// 说明:
//   你需要完成一个节点，它同时拥有一个发布者和一个订阅者。
//   发布者定时发送消息，订阅者接收并记录消息。
//   代码框架已经给出，你需要填写关键部分。
//
// 步骤:
//   1. 创建一个 Publisher，发布 std_msgs::msg::String 到话题 "ping"
//   2. 创建一个 Subscription，订阅话题 "ping"
//   3. 在定时回调中发布消息
//   4. 在订阅回调中记录收到的消息
//   5. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PubSubConnect : public rclcpp::Node
{
public:
  PubSubConnect() : Node("pubsub_connect"), count_(0), received_count_(0)
  {
    // TODO: 创建一个 Publisher，发布 std_msgs::msg::String 到话题 "ping"，QoS 深度 10
    // publisher_ = ???

    // TODO: 创建一个 Subscription，订阅话题 "ping"，QoS 深度 10
    // 回调函数为 sub_callback
    // subscription_ = ???

    // TODO: 创建一个定时器，每 100ms 调用 timer_callback
    // timer_ = ???
  }

  size_t get_received_count() const { return received_count_; }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Ping #" + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
    // TODO: 发布消息
    // ???
  }

  void sub_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received: '%s'", msg->data.c_str());
    // TODO: 增加 received_count_
    // ???
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
