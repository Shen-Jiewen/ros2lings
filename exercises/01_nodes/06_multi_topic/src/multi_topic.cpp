// I AM NOT DONE
//
// 练习: multi_topic
// 模块: 01 - Nodes & Topics
// 难度: ★★☆☆☆
//
// 学习目标:
//   学习在同一个节点中管理多个话题的发布者和订阅者。
//
// 说明:
//   你需要创建一个节点，拥有两个发布者和两个订阅者，
//   分别在不同的话题上工作。
//   - 话题 "status" 发布状态消息
//   - 话题 "command" 发布命令消息
//
// 步骤:
//   1. 创建两个 Publisher：一个发布到 "status"，一个发布到 "command"
//   2. 创建两个 Subscription：分别订阅 "status" 和 "command"
//   3. 在定时回调中向两个话题发布消息
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MultiTopicNode : public rclcpp::Node
{
public:
  MultiTopicNode() : Node("multi_topic_node"), count_(0)
  {
    // TODO: 创建第一个 Publisher，发布到话题 "status"，QoS 深度 10
    // status_pub_ = ???

    // TODO: 创建第二个 Publisher，发布到话题 "command"，QoS 深度 10
    // command_pub_ = ???

    // TODO: 创建第一个 Subscription，订阅话题 "status"
    // status_sub_ = ???

    // TODO: 创建第二个 Subscription，订阅话题 "command"
    // command_sub_ = ???

    timer_ = create_wall_timer(100ms, std::bind(&MultiTopicNode::timer_callback, this));
  }

  std::string get_last_status() const { return last_status_; }
  std::string get_last_command() const { return last_command_; }

private:
  void timer_callback()
  {
    // TODO: 创建并发布 status 消息
    // auto status_msg = std_msgs::msg::String();
    // status_msg.data = "Status: OK #" + std::to_string(count_);
    // status_pub_->publish(status_msg);

    // TODO: 创建并发布 command 消息
    // auto cmd_msg = std_msgs::msg::String();
    // cmd_msg.data = "Command: RUN #" + std::to_string(count_);
    // command_pub_->publish(cmd_msg);

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
