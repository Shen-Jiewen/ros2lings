#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class MultiNodeProcessTest : public ::testing::Test {
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

// 测试: MultiThreadedExecutor 可以管理多个节点
TEST_F(MultiNodeProcessTest, ExecutorManagesMultipleNodes) {
  auto node1 = std::make_shared<rclcpp::Node>("test_node_1");
  auto node2 = std::make_shared<rclcpp::Node>("test_node_2");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node1);
  executor.add_node(node2);

  // spin_some 应该能正常处理两个节点
  executor.spin_some();
  SUCCEED() << "MultiThreadedExecutor 成功管理多个节点";
}

// 测试: 同一进程中的两个节点可以通信
TEST_F(MultiNodeProcessTest, NodesInSameProcessCommunicate) {
  auto pub_node = std::make_shared<rclcpp::Node>("test_producer");
  auto sub_node = std::make_shared<rclcpp::Node>("test_consumer");

  bool message_received = false;
  std::string received_data;

  auto pub = pub_node->create_publisher<std_msgs::msg::String>("internal_topic", 10);
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
    "internal_topic", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      message_received = true;
      received_data = msg->data;
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  auto msg = std_msgs::msg::String();
  msg.data = "多节点测试消息";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!message_received &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(message_received) << "同一进程中的节点应能通过话题通信";
  EXPECT_EQ(received_data, "多节点测试消息");
}

// 测试: 验证两个节点的名称不同
TEST_F(MultiNodeProcessTest, NodesHaveDifferentNames) {
  auto node1 = std::make_shared<rclcpp::Node>("producer_node");
  auto node2 = std::make_shared<rclcpp::Node>("consumer_node");

  EXPECT_STREQ(node1->get_name(), "producer_node");
  EXPECT_STREQ(node2->get_name(), "consumer_node");
  EXPECT_STRNE(node1->get_name(), node2->get_name())
    << "两个节点应有不同的名称";
}
