#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_05_custom_message/msg/sensor_data.hpp>
#include <chrono>
#include <memory>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/custom_message.cpp"

using namespace std::chrono_literals;
using SensorData = ros2lings_05_custom_message::msg::SensorData;

class CustomMessageTest : public ::testing::Test {
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

TEST_F(CustomMessageTest, CanCreateNode) {
  auto node = std::make_shared<CustomMessageNode>();
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "custom_message_node");
}

TEST_F(CustomMessageTest, NodePublishesAndReceives) {
  auto node = std::make_shared<CustomMessageNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin for enough time for the timer (200ms) to fire and message to be received
  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < 1s) {
    executor.spin_some(50ms);
    if (!node->get_last_sensor_id().empty()) {
      break;
    }
  }

  EXPECT_EQ(node->get_last_sensor_id(), "sensor_01")
    << "Node should publish and receive SensorData with sensor_id='sensor_01'";
  EXPECT_DOUBLE_EQ(node->get_last_temperature(), 25.5)
    << "Node should receive temperature=25.5";
}

TEST_F(CustomMessageTest, CanCreateCustomMessage) {
  auto msg = SensorData();
  msg.temperature = 25.5;
  msg.humidity = 60.0;
  msg.sensor_id = "sensor_01";

  EXPECT_DOUBLE_EQ(msg.temperature, 25.5);
  EXPECT_DOUBLE_EQ(msg.humidity, 60.0);
  EXPECT_EQ(msg.sensor_id, "sensor_01");
}
