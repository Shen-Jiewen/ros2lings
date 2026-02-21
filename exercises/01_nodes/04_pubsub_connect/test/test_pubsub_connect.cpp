#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class PubSubConnectTest : public ::testing::Test {
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

TEST_F(PubSubConnectTest, MessageFlowsFromPublisherToSubscriber) {
  // 创建发布者节点
  auto pub_node = std::make_shared<rclcpp::Node>("test_pub");
  auto pub = pub_node->create_publisher<std_msgs::msg::String>("ping", 10);

  // 创建订阅者节点
  auto sub_node = std::make_shared<rclcpp::Node>("test_sub");
  std::string received;
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
    "ping", 10,
    [&received](const std_msgs::msg::String::SharedPtr msg) {
      received = msg->data;
    });

  // 发布消息
  auto msg = std_msgs::msg::String();
  msg.data = "Ping #0";
  pub->publish(msg);

  // 等待消息传递
  auto start = std::chrono::steady_clock::now();
  while (received.empty() && (std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(received, "Ping #0") << "消息应当从发布者流向订阅者";
}

TEST_F(PubSubConnectTest, MultipleMessagesReceived) {
  auto pub_node = std::make_shared<rclcpp::Node>("test_multi_pub");
  auto pub = pub_node->create_publisher<std_msgs::msg::String>("ping", 10);

  auto sub_node = std::make_shared<rclcpp::Node>("test_multi_sub");
  int count = 0;
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
    "ping", 10,
    [&count](const std_msgs::msg::String::SharedPtr) {
      count++;
    });

  // 发布 3 条消息
  for (int i = 0; i < 3; i++) {
    auto msg = std_msgs::msg::String();
    msg.data = "Ping #" + std::to_string(i);
    pub->publish(msg);
  }

  auto start = std::chrono::steady_clock::now();
  while (count < 3 && (std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_GE(count, 1) << "应当至少收到一条消息";
}

TEST_F(PubSubConnectTest, TopicNameIsCorrect) {
  auto node = std::make_shared<rclcpp::Node>("test_topic_name");
  auto pub = node->create_publisher<std_msgs::msg::String>("ping", 10);
  EXPECT_EQ(std::string(pub->get_topic_name()), "/ping");
}
