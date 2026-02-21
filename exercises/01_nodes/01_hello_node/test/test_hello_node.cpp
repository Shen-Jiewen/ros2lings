#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class HelloNodeTest : public ::testing::Test {
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

TEST_F(HelloNodeTest, CanCreateNode) {
  auto node = std::make_shared<rclcpp::Node>("test_hello_node");
  ASSERT_NE(node, nullptr);
}

TEST_F(HelloNodeTest, NodeHasCorrectName) {
  auto node = std::make_shared<rclcpp::Node>("test_hello_node");
  EXPECT_EQ(std::string(node->get_name()), "test_hello_node");
}

TEST_F(HelloNodeTest, CanSpinOnce) {
  auto node = std::make_shared<rclcpp::Node>("test_spin_node");
  rclcpp::spin_some(node);
  SUCCEED();
}
