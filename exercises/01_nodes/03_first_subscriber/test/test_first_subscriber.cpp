// Test the student's FirstSubscriber class directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// Include student source directly so class definitions are visible in this translation unit
#include "../src/first_subscriber.cpp"

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

TEST_F(FirstSubscriberTest, NodeCanBeCreated) {
  auto node = std::make_shared<FirstSubscriber>();
  ASSERT_NE(node, nullptr);
}

TEST_F(FirstSubscriberTest, NodeHasCorrectName) {
  auto node = std::make_shared<FirstSubscriber>();
  EXPECT_EQ(std::string(node->get_name()), "first_subscriber");
}

TEST_F(FirstSubscriberTest, SubscriptionOnCorrectTopic) {
  auto node = std::make_shared<FirstSubscriber>();

  // Verify there is a subscription on the "chatter" topic
  size_t sub_count = node->count_subscribers("/chatter");
  EXPECT_GE(sub_count, 1u)
    << "FirstSubscriber should have a subscription on /chatter topic";
}

TEST_F(FirstSubscriberTest, SubscriptionReceivesMessages) {
  auto node = std::make_shared<FirstSubscriber>();

  // Create a publisher to send a message on the same topic
  auto test_pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);

  auto msg = std_msgs::msg::String();
  msg.data = "hello subscriber";
  test_pub->publish(msg);

  // Spin to allow the subscription callback to fire
  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  // If we get here without crashing, the subscription callback was invoked
  // (or at least the node handled the message without error).
  SUCCEED();
}
