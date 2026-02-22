// Test the student's NamespaceRemapNode class directly.
// The student source file is compiled into this test binary via CMakeLists.txt.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// Include student source directly so class definitions are visible in this translation unit
#include "../src/namespace_remap.cpp"

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

TEST_F(NamespaceRemapTest, NodeCanBeCreated) {
  auto node = std::make_shared<NamespaceRemapNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(NamespaceRemapTest, NodeHasCorrectName) {
  auto node = std::make_shared<NamespaceRemapNode>();
  EXPECT_STREQ(node->get_name(), "data_publisher");
}

TEST_F(NamespaceRemapTest, NodeInCorrectNamespace) {
  auto node = std::make_shared<NamespaceRemapNode>();
  EXPECT_STREQ(node->get_namespace(), "/robot1")
    << "Node should be in the /robot1 namespace";
}

TEST_F(NamespaceRemapTest, TopicHasCorrectFullyQualifiedName) {
  auto node = std::make_shared<NamespaceRemapNode>();
  std::string topic_name = node->get_topic_name();
  EXPECT_EQ(topic_name, "/robot1/sensor_data")
    << "Topic fully-qualified name should be /robot1/sensor_data";
}
