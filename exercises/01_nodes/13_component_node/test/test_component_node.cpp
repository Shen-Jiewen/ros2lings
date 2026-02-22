// Test the student's TalkerComponent class directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

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

// The student's TalkerComponent must inherit rclcpp::Node and accept NodeOptions.
TEST_F(ComponentNodeTest, CanCreateWithNodeOptions) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<TalkerComponent>(options);
  ASSERT_NE(node, nullptr);
}

TEST_F(ComponentNodeTest, NodeHasCorrectName) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<TalkerComponent>(options);
  EXPECT_EQ(std::string(node->get_name()), "talker_component");
}

TEST_F(ComponentNodeTest, HasPublisherOnComponentChatter) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<TalkerComponent>(options);
  size_t pub_count = node->count_publishers("/component_chatter");
  EXPECT_GE(pub_count, 1u)
    << "TalkerComponent should have a publisher on /component_chatter topic";
}

TEST_F(ComponentNodeTest, PublishesMessages) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<TalkerComponent>(options);

  // Create a subscription to verify the component publishes
  bool message_received = false;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "component_chatter", 10,
    [&message_received](const std_msgs::msg::String::SharedPtr) {
      message_received = true;
    });

  // Spin to let the timer fire and publish a message
  auto start = std::chrono::steady_clock::now();
  while (!message_received &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(message_received)
    << "TalkerComponent should publish messages on component_chatter topic";
}
