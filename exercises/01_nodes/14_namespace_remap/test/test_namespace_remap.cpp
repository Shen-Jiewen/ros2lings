#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class NamespaceRemapTest : public ::testing::Test {
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

// 测试: 验证节点在正确的命名空间下
TEST_F(NamespaceRemapTest, NodeInCorrectNamespace) {
  auto node = std::make_shared<rclcpp::Node>("data_publisher", "robot1");
  EXPECT_STREQ(node->get_namespace(), "/robot1")
    << "节点应在 /robot1 命名空间下";
}

// 测试: 验证带命名空间的话题完全限定名
TEST_F(NamespaceRemapTest, TopicWithNamespace) {
  auto node = std::make_shared<rclcpp::Node>("data_publisher", "robot1");
  auto pub = node->create_publisher<std_msgs::msg::String>("sensor_data", 10);

  std::string topic_name = pub->get_topic_name();
  EXPECT_EQ(topic_name, "/robot1/sensor_data")
    << "话题全名应为 /robot1/sensor_data";
}

// 测试: 验证话题通信正常（使用正确的命名空间和话题名）
TEST_F(NamespaceRemapTest, CommunicationWorks) {
  auto pub_node = std::make_shared<rclcpp::Node>("data_publisher", "robot1");
  auto sub_node = std::make_shared<rclcpp::Node>("data_subscriber", "robot1");

  bool message_received = false;

  auto pub = pub_node->create_publisher<std_msgs::msg::String>("sensor_data", 10);
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
    "sensor_data", 10,
    [&](const std_msgs::msg::String::SharedPtr) {
      message_received = true;
    });

  auto msg = std_msgs::msg::String();
  msg.data = "test data";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!message_received &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(pub_node);
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(message_received) << "同一命名空间内的节点应能通信";
}
