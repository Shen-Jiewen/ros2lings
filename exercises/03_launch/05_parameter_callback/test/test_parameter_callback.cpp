#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <memory>
#include <vector>

class DynamicParamNode : public rclcpp::Node
{
public:
  DynamicParamNode() : Node("dynamic_param_node")
  {
    this->declare_parameter("speed", 10);
    speed_ = this->get_parameter("speed").as_int();

    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DynamicParamNode::param_callback, this, std::placeholders::_1));
  }

  int get_speed() const { return speed_; }

private:
  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      if (param.get_name() == "speed") {
        speed_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "speed changed to: %ld", param.as_int());
      }
    }

    return result;
  }

  int speed_ = 10;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

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
