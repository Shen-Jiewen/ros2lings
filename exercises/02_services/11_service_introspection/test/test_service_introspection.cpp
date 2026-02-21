#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

// 声明外部变量（来自 src/service_introspection.cpp 的全局变量）
extern std::string answer_list_services;
extern std::string answer_service_pattern;
extern std::string answer_call_command;
extern std::string answer_type_command;
extern std::string answer_guaranteed_response;

class ServiceIntrospectionTest : public ::testing::Test {
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

TEST_F(ServiceIntrospectionTest, ListServicesCommand) {
  EXPECT_EQ(answer_list_services, "ros2 service list")
    << "列出所有活跃服务的命令是 ros2 service list";
}

TEST_F(ServiceIntrospectionTest, ServicePattern) {
  EXPECT_EQ(answer_service_pattern, "request_response")
    << "Service 使用请求-响应 (request_response) 模式";
}

TEST_F(ServiceIntrospectionTest, CallCommand) {
  EXPECT_EQ(answer_call_command, "ros2 service call")
    << "从命令行调用服务的命令是 ros2 service call";
}

TEST_F(ServiceIntrospectionTest, TypeCommand) {
  EXPECT_EQ(answer_type_command, "ros2 service type")
    << "查看服务类型的命令是 ros2 service type";
}

TEST_F(ServiceIntrospectionTest, GuaranteedResponse) {
  EXPECT_EQ(answer_guaranteed_response, "yes")
    << "Service 保证请求会被响应（服务端正常运行时）";
}
