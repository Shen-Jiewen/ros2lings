// Test the student's QosMismatchNode class directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// Include student source directly so class definitions are visible in this translation unit
#include "../src/qos_mismatch.cpp"

class QosMismatchTest : public ::testing::Test {
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

TEST_F(QosMismatchTest, NodeCanBeCreated) {
  auto node = std::make_shared<QosMismatchNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(QosMismatchTest, NodeHasCorrectName) {
  auto node = std::make_shared<QosMismatchNode>();
  EXPECT_EQ(std::string(node->get_name()), "qos_mismatch_node");
}

TEST_F(QosMismatchTest, HasPublisherAndSubscription) {
  auto node = std::make_shared<QosMismatchNode>();

  size_t pub_count = node->count_publishers("/qos_test_topic");
  EXPECT_GE(pub_count, 1u)
    << "QosMismatchNode should have a publisher on /qos_test_topic";

  size_t sub_count = node->count_subscribers("/qos_test_topic");
  EXPECT_GE(sub_count, 1u)
    << "QosMismatchNode should have a subscription on /qos_test_topic";
}

// The key test: if QoS is compatible, the node's internal pub/sub will
// communicate and received_count will increase. If QoS is mismatched,
// received_count stays at 0.
TEST_F(QosMismatchTest, QosCompatibleMessagesReceived) {
  auto node = std::make_shared<QosMismatchNode>();
  EXPECT_EQ(node->get_received_count(), 0);

  // Spin the node to let the timer fire and the internal pub/sub communicate
  auto start = std::chrono::steady_clock::now();
  while (node->get_received_count() == 0 &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_GT(node->get_received_count(), 0)
    << "QoS must be compatible for messages to be received. "
       "received_count should increase when pub/sub QoS settings are compatible.";
}
