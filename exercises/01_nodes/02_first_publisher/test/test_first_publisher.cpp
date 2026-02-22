// Test the student's FirstPublisher class directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// Forward-declare nothing: FirstPublisher is defined in the compiled source.
// The class definition comes from src/first_publisher.cpp which is compiled
// into this test binary with ROS2LINGS_TEST defined (main is guarded out).

class FirstPublisherTest : public ::testing::Test {
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

TEST_F(FirstPublisherTest, NodeCanBeCreated) {
  auto node = std::make_shared<FirstPublisher>();
  ASSERT_NE(node, nullptr);
}

TEST_F(FirstPublisherTest, NodeHasCorrectName) {
  auto node = std::make_shared<FirstPublisher>();
  EXPECT_EQ(std::string(node->get_name()), "first_publisher");
}

TEST_F(FirstPublisherTest, PublisherOnCorrectTopic) {
  auto node = std::make_shared<FirstPublisher>();

  // Check that a publisher exists on the "chatter" topic
  auto topic_names_and_types = node->get_topic_names_and_types();
  bool found = false;
  for (const auto & entry : topic_names_and_types) {
    if (entry.first == "/chatter") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "FirstPublisher should have a publisher on /chatter topic";
}

TEST_F(FirstPublisherTest, PublisherCountOnChatter) {
  auto node = std::make_shared<FirstPublisher>();
  size_t pub_count = node->count_publishers("/chatter");
  EXPECT_GE(pub_count, 1u) << "There should be at least one publisher on /chatter";
}
