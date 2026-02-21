#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_05_custom_message/msg/sensor_data.hpp>
#include <chrono>
#include <memory>

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

TEST_F(CustomMessageTest, CanCreateCustomMessage) {
  auto msg = SensorData();
  msg.temperature = 25.5;
  msg.humidity = 60.0;
  msg.sensor_id = "sensor_01";

  EXPECT_DOUBLE_EQ(msg.temperature, 25.5);
  EXPECT_DOUBLE_EQ(msg.humidity, 60.0);
  EXPECT_EQ(msg.sensor_id, "sensor_01");
}

TEST_F(CustomMessageTest, CanPublishCustomMessage) {
  auto node = std::make_shared<rclcpp::Node>("test_custom_pub");
  auto pub = node->create_publisher<SensorData>("sensor_data", 10);
  ASSERT_NE(pub, nullptr);

  auto msg = SensorData();
  msg.temperature = 25.5;
  msg.humidity = 60.0;
  msg.sensor_id = "sensor_01";
  pub->publish(msg);
  SUCCEED();
}

TEST_F(CustomMessageTest, CanSubscribeCustomMessage) {
  auto pub_node = std::make_shared<rclcpp::Node>("test_custom_pub2");
  auto sub_node = std::make_shared<rclcpp::Node>("test_custom_sub");

  auto pub = pub_node->create_publisher<SensorData>("sensor_data", 10);

  std::string received_id;
  double received_temp = 0.0;
  auto sub = sub_node->create_subscription<SensorData>(
    "sensor_data", 10,
    [&received_id, &received_temp](const SensorData::SharedPtr msg) {
      received_id = msg->sensor_id;
      received_temp = msg->temperature;
    });

  auto msg = SensorData();
  msg.temperature = 25.5;
  msg.humidity = 60.0;
  msg.sensor_id = "sensor_01";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (received_id.empty() && (std::chrono::steady_clock::now() - start) < 2s) {
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(received_id, "sensor_01") << "应当收到自定义消息并读取 sensor_id 字段";
  EXPECT_DOUBLE_EQ(received_temp, 25.5) << "应当收到自定义消息并读取 temperature 字段";
}
