// Test the student's MultiTopicNode class directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

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

TEST_F(MultiTopicTest, NodeCanBeCreated) {
  auto node = std::make_shared<MultiTopicNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(MultiTopicTest, NodeHasCorrectName) {
  auto node = std::make_shared<MultiTopicNode>();
  EXPECT_EQ(std::string(node->get_name()), "multi_topic_node");
}

TEST_F(MultiTopicTest, HasPublisherOnStatusTopic) {
  auto node = std::make_shared<MultiTopicNode>();
  size_t pub_count = node->count_publishers("/status");
  EXPECT_GE(pub_count, 1u)
    << "MultiTopicNode should have a publisher on /status topic";
}

TEST_F(MultiTopicTest, HasPublisherOnCommandTopic) {
  auto node = std::make_shared<MultiTopicNode>();
  size_t pub_count = node->count_publishers("/command");
  EXPECT_GE(pub_count, 1u)
    << "MultiTopicNode should have a publisher on /command topic";
}

TEST_F(MultiTopicTest, HasSubscriptionOnStatusTopic) {
  auto node = std::make_shared<MultiTopicNode>();
  size_t sub_count = node->count_subscribers("/status");
  EXPECT_GE(sub_count, 1u)
    << "MultiTopicNode should have a subscription on /status topic";
}

TEST_F(MultiTopicTest, HasSubscriptionOnCommandTopic) {
  auto node = std::make_shared<MultiTopicNode>();
  size_t sub_count = node->count_subscribers("/command");
  EXPECT_GE(sub_count, 1u)
    << "MultiTopicNode should have a subscription on /command topic";
}

TEST_F(MultiTopicTest, TopicsAreIndependent) {
  auto node = std::make_shared<MultiTopicNode>();
  // Verify the two topics have different names
  size_t status_pub = node->count_publishers("/status");
  size_t command_pub = node->count_publishers("/command");
  EXPECT_GE(status_pub, 1u);
  EXPECT_GE(command_pub, 1u);
}

TEST_F(MultiTopicTest, MessagesFlowOnBothTopics) {
  auto node = std::make_shared<MultiTopicNode>();

  // Spin the node to let the timer fire and the internal pub/sub communicate
  auto start = std::chrono::steady_clock::now();
  bool status_received = false;
  bool command_received = false;

  while ((!status_received || !command_received) &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);

    if (!node->get_last_status().empty()) {
      status_received = true;
    }
    if (!node->get_last_command().empty()) {
      command_received = true;
    }
  }

  EXPECT_TRUE(status_received)
    << "Should receive messages on the status topic";
  EXPECT_TRUE(command_received)
    << "Should receive messages on the command topic";
}
