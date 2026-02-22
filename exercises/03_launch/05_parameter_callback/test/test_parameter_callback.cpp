#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <memory>
#include <vector>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/parameter_callback.cpp"

class ParameterCallbackTest : public ::testing::Test {
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

TEST_F(ParameterCallbackTest, CanCreateNode) {
  auto node = std::make_shared<DynamicParamNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(ParameterCallbackTest, DefaultSpeedIs10) {
  auto node = std::make_shared<DynamicParamNode>();
  EXPECT_EQ(node->get_speed(), 10);
}

TEST_F(ParameterCallbackTest, CanSetParameter) {
  auto node = std::make_shared<DynamicParamNode>();
  auto result = node->set_parameter(rclcpp::Parameter("speed", 20));
  EXPECT_TRUE(result.successful);
}

TEST_F(ParameterCallbackTest, CallbackUpdatesInternalState) {
  auto node = std::make_shared<DynamicParamNode>();
  node->set_parameter(rclcpp::Parameter("speed", 42));
  EXPECT_EQ(node->get_speed(), 42) << "回调应更新内部 speed_ 值";
}

TEST_F(ParameterCallbackTest, ParameterValuePersists) {
  auto node = std::make_shared<DynamicParamNode>();
  node->set_parameter(rclcpp::Parameter("speed", 99));
  auto param = node->get_parameter("speed");
  EXPECT_EQ(param.as_int(), 99) << "参数值应更新为 99";
}
