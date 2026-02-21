cmake_minimum_required(VERSION 3.8)
project(ros2lings_22_action_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2lings_interfaces REQUIRED)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action ros2lings_interfaces)
install(TARGETS action_client DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_client test/test_action_client.cpp)
  ament_target_dependencies(test_action_client rclcpp rclcpp_action ros2lings_interfaces)
endif()

ament_package()
