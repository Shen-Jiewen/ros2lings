#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_19_custom_srv/srv/compute_area.hpp>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;
using ComputeArea = ros2lings_19_custom_srv::srv::ComputeArea;

class CustomSrvTest : public ::testing::Test {
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

TEST_F(CustomSrvTest, RequestFieldsExist) {
  auto request = std::make_shared<ComputeArea::Request>();
  request->width = 5.0;
  request->height = 3.0;
  EXPECT_DOUBLE_EQ(request->width, 5.0);
  EXPECT_DOUBLE_EQ(request->height, 3.0);
}

TEST_F(CustomSrvTest, ResponseFieldExists) {
  auto response = std::make_shared<ComputeArea::Response>();
  response->area = 15.0;
  EXPECT_DOUBLE_EQ(response->area, 15.0);
}

TEST_F(CustomSrvTest, ServiceComputesCorrectArea) {
  auto node = std::make_shared<rclcpp::Node>("test_area_node");

  // 创建服务器
  auto service = node->create_service<ComputeArea>(
    "compute_area",
    [](const std::shared_ptr<ComputeArea::Request> request,
       std::shared_ptr<ComputeArea::Response> response) {
      response->area = request->width * request->height;
    });

  // 创建客户端
  auto client = node->create_client<ComputeArea>("compute_area");
  ASSERT_TRUE(client->wait_for_service(2s));

  // 测试正常计算
  auto request = std::make_shared<ComputeArea::Request>();
  request->width = 3.5;
  request->height = 4.2;
  auto future = client->async_send_request(request);

  auto status = rclcpp::spin_until_future_complete(node, future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_NEAR(response->area, 14.7, 0.001) << "3.5 * 4.2 应当约等于 14.7";
}

TEST_F(CustomSrvTest, ServiceHandlesZero) {
  auto node = std::make_shared<rclcpp::Node>("test_area_zero");

  auto service = node->create_service<ComputeArea>(
    "compute_area_zero",
    [](const std::shared_ptr<ComputeArea::Request> request,
       std::shared_ptr<ComputeArea::Response> response) {
      response->area = request->width * request->height;
    });

  auto client = node->create_client<ComputeArea>("compute_area_zero");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<ComputeArea::Request>();
  request->width = 0.0;
  request->height = 10.0;
  auto future = client->async_send_request(request);

  auto status = rclcpp::spin_until_future_complete(node, future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_DOUBLE_EQ(response->area, 0.0) << "宽为 0 时面积应为 0";
}
