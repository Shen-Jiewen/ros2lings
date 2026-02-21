#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class FirstSubscriberTest : public ::testing::Test {
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

TEST_F(FirstSubscriberTest, CanCreateSubscription) {
  auto node = std::make_shared<rclcpp::Node>("test_sub_create");
  bool callback_called = false;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    [&callback_called](const std_msgs::msg::String::SharedPtr) {
      callback_called = true;
    });
  ASSERT_NE(sub, nullptr);
}

TEST_F(FirstSubscriberTest, SubscriptionOnCorrectTopic) {
  auto node = std::make_shared<rclcpp::Node>("test_sub_topic");
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    [](const std_msgs::msg::String::SharedPtr) {});
  EXPECT_EQ(std::string(sub->get_topic_name()), "/chatter");
}

TEST_F(FirstSubscriberTest, CanReceiveMessage) {
  auto pub_node = std::make_shared<rclcpp::Node>("test_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("test_sub");

  auto pub = pub_node->create_publisher<std_msgs::msg::String>("chatter", 10);

  std::string received_data;
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    [&received_data](const std_msgs::msg::String::SharedPtr msg) {
      received_data = msg->data;
    });

  // 发布消息
  auto message = std_msgs::msg::String();
  message.data = "hello subscriber";
  pub->publish(message);

  // 等待接收
  auto start = std::chrono::steady_clock::now();
  while (received_data.empty() && (std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(received_data, "hello subscriber") << "订阅者应当收到发布的消息";
}
