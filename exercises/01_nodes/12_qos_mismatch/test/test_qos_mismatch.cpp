#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class QosMismatchTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// 测试: 发布者和订阅者的 QoS 兼容，消息可以正常传递
TEST_F(QosMismatchTest, PubSubCommunicate) {
  auto node = std::make_shared<rclcpp::Node>("test_qos_node");
  bool message_received = false;

  // 创建兼容的 QoS：两端都使用 RELIABLE + TRANSIENT_LOCAL
  auto pub_qos = rclcpp::QoS(10);
  pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto sub_qos = rclcpp::QoS(10);
  sub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  sub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto publisher = node->create_publisher<std_msgs::msg::String>("test_qos_topic", pub_qos);

  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "test_qos_topic", sub_qos,
    [&message_received](const std_msgs::msg::String::SharedPtr) {
      message_received = true;
    });

  // 发布消息并 spin
  auto msg = std_msgs::msg::String();
  msg.data = "hello qos";
  publisher->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!message_received &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(message_received) << "QoS 兼容后应能收到消息";
}

// 测试: RELIABLE 发布者 + BEST_EFFORT 订阅者也应兼容
TEST_F(QosMismatchTest, ReliablePubBestEffortSubCompatible) {
  auto node = std::make_shared<rclcpp::Node>("test_qos_compat_node");
  bool message_received = false;

  auto pub_qos = rclcpp::QoS(10).reliable();
  auto sub_qos = rclcpp::QoS(10).best_effort();

  auto publisher = node->create_publisher<std_msgs::msg::String>("test_compat_topic", pub_qos);
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "test_compat_topic", sub_qos,
    [&message_received](const std_msgs::msg::String::SharedPtr) {
      message_received = true;
    });

  auto msg = std_msgs::msg::String();
  msg.data = "compat test";
  publisher->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!message_received &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(message_received) << "RELIABLE pub + BEST_EFFORT sub 应兼容";
}

// 测试: 创建 QoS 对象并验证策略设置
TEST_F(QosMismatchTest, QosPolicySettings) {
  auto qos = rclcpp::QoS(10);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto profile = qos.get_rmw_qos_profile();
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}
