#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class MultiTopicTest : public ::testing::Test {
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

TEST_F(MultiTopicTest, CanPublishToStatusTopic) {
  auto node = std::make_shared<rclcpp::Node>("test_status");
  auto pub = node->create_publisher<std_msgs::msg::String>("status", 10);

  bool received = false;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "status", 10,
    [&received](const std_msgs::msg::String::SharedPtr) { received = true; });

  auto msg = std_msgs::msg::String();
  msg.data = "Status: OK #0";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!received && (std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(received) << "应当在 status 话题上收到消息";
}

TEST_F(MultiTopicTest, CanPublishToCommandTopic) {
  auto node = std::make_shared<rclcpp::Node>("test_command");
  auto pub = node->create_publisher<std_msgs::msg::String>("command", 10);

  bool received = false;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "command", 10,
    [&received](const std_msgs::msg::String::SharedPtr) { received = true; });

  auto msg = std_msgs::msg::String();
  msg.data = "Command: RUN #0";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!received && (std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(received) << "应当在 command 话题上收到消息";
}

TEST_F(MultiTopicTest, TopicsAreIndependent) {
  auto node = std::make_shared<rclcpp::Node>("test_independent");
  auto status_pub = node->create_publisher<std_msgs::msg::String>("status", 10);
  auto command_pub = node->create_publisher<std_msgs::msg::String>("command", 10);

  EXPECT_NE(std::string(status_pub->get_topic_name()),
            std::string(command_pub->get_topic_name()))
    << "两个话题应当有不同的名称";

  EXPECT_EQ(std::string(status_pub->get_topic_name()), "/status");
  EXPECT_EQ(std::string(command_pub->get_topic_name()), "/command");
}
