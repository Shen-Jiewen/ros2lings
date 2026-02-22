// Test the student's PubSubConnect class directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

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

TEST_F(PubSubConnectTest, NodeCanBeCreated) {
  auto node = std::make_shared<PubSubConnect>();
  ASSERT_NE(node, nullptr);
}

TEST_F(PubSubConnectTest, NodeHasCorrectName) {
  auto node = std::make_shared<PubSubConnect>();
  EXPECT_EQ(std::string(node->get_name()), "pubsub_connect");
}

TEST_F(PubSubConnectTest, HasPublisherOnPingTopic) {
  auto node = std::make_shared<PubSubConnect>();
  size_t pub_count = node->count_publishers("/ping");
  EXPECT_GE(pub_count, 1u)
    << "PubSubConnect should have a publisher on /ping topic";
}

TEST_F(PubSubConnectTest, HasSubscriptionOnPingTopic) {
  auto node = std::make_shared<PubSubConnect>();
  size_t sub_count = node->count_subscribers("/ping");
  EXPECT_GE(sub_count, 1u)
    << "PubSubConnect should have a subscription on /ping topic";
}

TEST_F(PubSubConnectTest, ReceivedCountIncreases) {
  auto node = std::make_shared<PubSubConnect>();
  EXPECT_EQ(node->get_received_count(), 0u);

  // Spin the node to let the timer fire and the internal pub/sub communicate
  auto start = std::chrono::steady_clock::now();
  while (node->get_received_count() == 0 &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_GT(node->get_received_count(), 0u)
    << "received_count should increase as the node publishes and receives messages";
}
