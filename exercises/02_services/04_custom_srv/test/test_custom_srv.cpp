#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_19_custom_srv/srv/compute_area.hpp>
#include <chrono>
#include <memory>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/custom_srv.cpp"

// ComputeArea alias comes from the included student source
using namespace std::chrono_literals;

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

TEST_F(CustomSrvTest, CanCreateServer) {
  auto server = std::make_shared<AreaServer>();
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(std::string(server->get_name()), "area_server");
}

TEST_F(CustomSrvTest, ServiceComputesCorrectArea) {
  auto server_node = std::make_shared<AreaServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_area_client");

  auto client = client_node->create_client<ComputeArea>("compute_area");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_service(2s))
    << "Student's 'compute_area' service should be available";

  auto request = std::make_shared<ComputeArea::Request>();
  request->width = 3.5;
  request->height = 4.2;
  auto future = client->async_send_request(request);

  auto status = executor.spin_until_future_complete(future, 5s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_NEAR(response->area, 14.7, 0.001) << "3.5 * 4.2 should be approximately 14.7";
}

TEST_F(CustomSrvTest, ServiceHandlesZero) {
  auto server_node = std::make_shared<AreaServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_area_zero_client");

  auto client = client_node->create_client<ComputeArea>("compute_area");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<ComputeArea::Request>();
  request->width = 0.0;
  request->height = 10.0;
  auto future = client->async_send_request(request);

  auto status = executor.spin_until_future_complete(future, 5s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_DOUBLE_EQ(response->area, 0.0) << "Area should be 0 when width is 0";
}
