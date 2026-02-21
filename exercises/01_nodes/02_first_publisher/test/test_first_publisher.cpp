#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class FirstPublisherTest : public ::testing::Test {
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

TEST_F(FirstPublisherTest, NodeCanBeCreated) {
  auto node = std::make_shared<rclcpp::Node>("test_pub_node");
  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
  ASSERT_NE(pub, nullptr);
}

TEST_F(FirstPublisherTest, PublisherOnCorrectTopic) {
  auto node = std::make_shared<rclcpp::Node>("test_pub_topic");
  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
  EXPECT_EQ(pub->get_topic_name(), std::string("/chatter"));
}

TEST_F(FirstPublisherTest, CanPublishMessage) {
  auto pub_node = std::make_shared<rclcpp::Node>("test_publisher");
  auto sub_node = std::make_shared<rclcpp::Node>("test_subscriber");

  auto pub = pub_node->create_publisher<std_msgs::msg::String>("chatter", 10);

  bool received = false;
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    [&received](const std_msgs::msg::String::SharedPtr msg) {
      (void)msg;
      received = true;
    });

  // 发布一条消息
  auto message = std_msgs::msg::String();
  message.data = "test message";
  pub->publish(message);

  // spin 让消息传递
  auto start = std::chrono::steady_clock::now();
  while (!received && (std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(received) << "应当接收到发布的消息";
}
