// Test the student's ProducerNode and ConsumerNode classes directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

// Include student source directly so class definitions are visible in this translation unit
#include "../src/multi_node_process.cpp"

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

TEST_F(MultiNodeProcessTest, ProducerNodeCanBeCreated) {
  auto producer = std::make_shared<ProducerNode>();
  ASSERT_NE(producer, nullptr);
  EXPECT_STREQ(producer->get_name(), "producer_node");
}

TEST_F(MultiNodeProcessTest, ConsumerNodeCanBeCreated) {
  auto consumer = std::make_shared<ConsumerNode>();
  ASSERT_NE(consumer, nullptr);
  EXPECT_STREQ(consumer->get_name(), "consumer_node");
}

TEST_F(MultiNodeProcessTest, NodesHaveDifferentNames) {
  auto producer = std::make_shared<ProducerNode>();
  auto consumer = std::make_shared<ConsumerNode>();
  EXPECT_STRNE(producer->get_name(), consumer->get_name())
    << "Producer and consumer should have different node names";
}

TEST_F(MultiNodeProcessTest, ProducerPublishesOnInternalTopic) {
  auto producer = std::make_shared<ProducerNode>();
  size_t pub_count = producer->count_publishers("/internal_topic");
  EXPECT_GE(pub_count, 1u)
    << "ProducerNode should have a publisher on /internal_topic";
}

TEST_F(MultiNodeProcessTest, ConsumerSubscribesToInternalTopic) {
  auto consumer = std::make_shared<ConsumerNode>();
  size_t sub_count = consumer->count_subscribers("/internal_topic");
  EXPECT_GE(sub_count, 1u)
    << "ConsumerNode should have a subscription on /internal_topic";
}

TEST_F(MultiNodeProcessTest, ExecutorManagesBothNodes) {
  auto producer = std::make_shared<ProducerNode>();
  auto consumer = std::make_shared<ConsumerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(producer);
  executor.add_node(consumer);

  // Spin to let the producer timer fire and consumer receive messages
  auto start = std::chrono::steady_clock::now();
  while (consumer->get_received_count() == 0 &&
         (std::chrono::steady_clock::now() - start) < 3s)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_GT(consumer->get_received_count(), 0)
    << "Consumer should receive messages from producer via the executor";
}
