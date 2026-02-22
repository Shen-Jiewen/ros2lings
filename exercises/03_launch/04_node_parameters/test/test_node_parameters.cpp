#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class ParameterNode : public rclcpp::Node
{
public:
  ParameterNode() : Node("parameter_node")
  {
    this->declare_parameter("max_speed", 10);
    this->declare_parameter("robot_name", "ros2bot");
    this->declare_parameter("update_frequency", 30.0);

    speed_ = this->get_parameter("max_speed").as_int();
    name_ = this->get_parameter("robot_name").as_string();
    freq_ = this->get_parameter("update_frequency").as_double();
  }

  int get_speed() const { return speed_; }
  std::string get_name_value() const { return name_; }
  double get_frequency() const { return freq_; }

private:
  int speed_ = 0;
  std::string name_;
  double freq_ = 0.0;
};

class NodeParametersTest : public ::testing::Test {
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

TEST_F(NodeParametersTest, CanCreateNode) {
  auto node = std::make_shared<ParameterNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(NodeParametersTest, MaxSpeedDefaultValue) {
  auto node = std::make_shared<ParameterNode>();
  EXPECT_EQ(node->get_speed(), 10);
}

TEST_F(NodeParametersTest, MaxSpeedIsInteger) {
  auto node = std::make_shared<ParameterNode>();
  auto param = node->get_parameter("max_speed");
  EXPECT_EQ(param.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
}

TEST_F(NodeParametersTest, RobotNameDefaultValue) {
  auto node = std::make_shared<ParameterNode>();
  EXPECT_EQ(node->get_name_value(), "ros2bot");
}

TEST_F(NodeParametersTest, RobotNameIsString) {
  auto node = std::make_shared<ParameterNode>();
  auto param = node->get_parameter("robot_name");
  EXPECT_EQ(param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
}

TEST_F(NodeParametersTest, UpdateFrequencyDefaultValue) {
  auto node = std::make_shared<ParameterNode>();
  EXPECT_DOUBLE_EQ(node->get_frequency(), 30.0);
}

TEST_F(NodeParametersTest, UpdateFrequencyIsDouble) {
  auto node = std::make_shared<ParameterNode>();
  auto param = node->get_parameter("update_frequency");
  EXPECT_EQ(param.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE);
}

TEST_F(NodeParametersTest, ParameterNamesCorrect) {
  auto node = std::make_shared<ParameterNode>();
  EXPECT_TRUE(node->has_parameter("max_speed"));
  EXPECT_TRUE(node->has_parameter("robot_name"));
  EXPECT_TRUE(node->has_parameter("update_frequency"));
}
