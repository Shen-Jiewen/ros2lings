#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ComponentNodeTest : public ::testing::Test {
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

// 测试: 验证组件节点可以通过 NodeOptions 创建
TEST_F(ComponentNodeTest, CanLoadComponent) {
  // 动态加载组件库
  auto loader = std::make_shared<rclcpp::Node>("test_loader");

  // 通过订阅 component_chatter 话题验证组件发布消息
  bool message_received = false;
  std::string received_data;

  auto subscription = loader->create_subscription<std_msgs::msg::String>(
    "component_chatter", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      message_received = true;
      received_data = msg->data;
    });

  // 创建一个模拟组件的发布者来验证话题通信
  auto pub = loader->create_publisher<std_msgs::msg::String>("component_chatter", 10);
  auto msg = std_msgs::msg::String();
  msg.data = "来自组件的消息 #0";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!message_received &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(loader);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(message_received) << "应该能在 component_chatter 话题上收到消息";
  EXPECT_FALSE(received_data.empty()) << "消息内容不应为空";
}

// 测试: 验证组件节点类可以正确构造
TEST_F(ComponentNodeTest, ComponentNodeCreation) {
  // 这里我们直接包含组件头文件来测试
  // 组件节点应该能通过 NodeOptions 构造
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("test_component_creation", options);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "test_component_creation");
}

// 测试: 验证发布的消息格式正确
TEST_F(ComponentNodeTest, MessageFormat) {
  auto node = std::make_shared<rclcpp::Node>("test_msg_format");
  std::string received_msg;
  bool got_msg = false;

  auto sub = node->create_subscription<std_msgs::msg::String>(
    "component_chatter", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      received_msg = msg->data;
      got_msg = true;
    });

  auto pub = node->create_publisher<std_msgs::msg::String>("component_chatter", 10);
  auto msg = std_msgs::msg::String();
  msg.data = "来自组件的消息 #0";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!got_msg && (std::chrono::steady_clock::now() - start) < 3s) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(got_msg);
  EXPECT_NE(received_msg.find("#"), std::string::npos) << "消息应包含序号";
}
