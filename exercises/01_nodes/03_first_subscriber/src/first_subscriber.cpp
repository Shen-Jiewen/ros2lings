// I AM NOT DONE
//
// 练习: first_subscriber
// 模块: 01 - Nodes & Topics
// 难度: ★☆☆☆☆
//
// 学习目标:
//   理解 ROS2 中 Subscription 的回调绑定和 QoS 配置。
//
// 说明:
//   下面的代码尝试创建一个订阅者节点，从话题接收字符串消息。
//   但代码中有几个错误需要你修复。
//
// 步骤:
//   1. 修复回调函数的参数签名
//   2. 添加缺失的 QoS 参数
//   3. 修复话题名称使其与发布者匹配
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class FirstSubscriber : public rclcpp::Node
{
public:
  FirstSubscriber() : Node("first_subscriber")
  {
    // TODO: create_subscription 需要 QoS 参数，且话题名称应该是 "chatter"
    // 提示: create_subscription<消息类型>("话题名", QoS深度, 回调)
    subscription_ = create_subscription<std_msgs::msg::String>(
      "wrong_topic",
      std::bind(&FirstSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  // TODO: 回调函数参数应该是 SharedPtr 类型
  // 提示: const std_msgs::msg::String::SharedPtr msg
  void topic_callback(const std_msgs::msg::String msg)
  {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str());
    last_msg_ = msg.data;
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
