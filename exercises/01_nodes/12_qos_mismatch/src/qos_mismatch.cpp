// I AM NOT DONE
//
// 练习: qos_mismatch
// 模块: 01 - Nodes & Topics
// 难度: ★★☆☆☆
//
// 学习目标:
//   诊断并修复 QoS（服务质量）不兼容问题，使发布者和订阅者能正常通信。
//
// 说明:
//   下面的代码中，发布者和订阅者的 QoS 设置不兼容，导致消息无法传递。
//   你需要修复 QoS 配置，使两端能够成功通信。
//
// 背景知识:
//   - QoS 的 reliability 策略: RELIABLE 要求确认收到，BEST_EFFORT 不需要
//   - QoS 的 durability 策略: TRANSIENT_LOCAL 会缓存消息，VOLATILE 不缓存
//   - 兼容规则: 发布者的 QoS 必须 "至少" 满足订阅者的要求
//   - RELIABLE 发布者 + RELIABLE 订阅者 = 兼容
//   - RELIABLE 发布者 + BEST_EFFORT 订阅者 = 兼容
//   - BEST_EFFORT 发布者 + RELIABLE 订阅者 = 不兼容!
//   - TRANSIENT_LOCAL 发布者 + TRANSIENT_LOCAL 订阅者 = 兼容
//   - VOLATILE 发布者 + TRANSIENT_LOCAL 订阅者 = 不兼容!
//
// 步骤:
//   1. 分析发布者和订阅者的 QoS 设置
//   2. 找出不兼容的地方
//   3. 修复 QoS 使两端兼容（最简单的方式是让两端匹配）
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class QosMismatchNode : public rclcpp::Node
{
public:
  QosMismatchNode() : Node("qos_mismatch_node"), received_count_(0)
  {
    // 发布者 QoS: BEST_EFFORT 可靠性，VOLATILE 持久性
    // BUG: 发布者使用 BEST_EFFORT，但订阅者期望 RELIABLE，不兼容！
    auto pub_qos = rclcpp::QoS(10);
    pub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    pub_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    publisher_ = this->create_publisher<std_msgs::msg::String>("qos_test_topic", pub_qos);

    // 订阅者 QoS: RELIABLE 可靠性，TRANSIENT_LOCAL 持久性
    // BUG: 订阅者要求 TRANSIENT_LOCAL，但发布者只提供 VOLATILE，不兼容！
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
