// I AM NOT DONE
//
// 练习: first_publisher
// 模块: 01 - Nodes & Topics
// 难度: ★☆☆☆☆
//
// 学习目标:
//   理解 ROS2 中 Publisher 的创建和 Timer 回调机制。
//
// 说明:
//   下面的代码尝试创建一个发布者节点，每秒向话题发布一条字符串消息。
//   但代码中有几个错误需要你修复。
//
// 步骤:
//   1. 修复 create_publisher 的调用语法
//   2. 修复 Timer 的周期参数类型
//   3. 补充消息发布调用
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class FirstPublisher : public rclcpp::Node
{
public:
  FirstPublisher() : Node("first_publisher"), count_(0)
  {
    // TODO: create_publisher 需要模板参数和两个参数：话题名称和 QoS 深度
    // 提示: create_publisher<消息类型>("话题名", QoS深度)
    publisher_ = create_publisher("chatter");

    // TODO: 定时器周期需要使用 std::chrono 的时间类型，而不是整数
    // 提示: 使用 500ms 或 std::chrono::milliseconds(500)
    timer_ = create_wall_timer(500, std::bind(&FirstPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2! Count: " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());

    // TODO: 使用 publisher_ 发布 message
    // 提示: publisher_->???
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirstPublisher>());
  rclcpp::shutdown();
  return 0;
}
#endif
