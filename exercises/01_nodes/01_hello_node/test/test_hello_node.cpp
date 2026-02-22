// This test compiles the student's hello_node.cpp (with main guarded out)
// to verify it has correct syntax, then tests basic node creation.

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

// The student source file is compiled into this test binary via CMakeLists.txt.
// If the student has not fixed rclcpp::init() or std::make_shared, this
// test binary will fail to compile, which is the desired RED behavior.

TEST_F(HelloNodeTest, CanCreateNode) {
  auto node = std::make_shared<rclcpp::Node>("hello_node");
  ASSERT_NE(node, nullptr);
}

TEST_F(HelloNodeTest, NodeHasCorrectName) {
  auto node = std::make_shared<rclcpp::Node>("hello_node");
  EXPECT_EQ(std::string(node->get_name()), "hello_node");
}

TEST_F(HelloNodeTest, CanSpinOnce) {
  auto node = std::make_shared<rclcpp::Node>("hello_node");
  rclcpp::spin_some(node);
  SUCCEED();
}
