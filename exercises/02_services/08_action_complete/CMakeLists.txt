cmake_minimum_required(VERSION 3.8)
project(ros2lings_23_action_complete)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2lings_interfaces REQUIRED)

add_executable(action_complete src/action_complete.cpp)
ament_target_dependencies(action_complete rclcpp rclcpp_action ros2lings_interfaces)
install(TARGETS action_complete DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_complete test/test_action_complete.cpp)
  target_compile_definitions(test_action_complete PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_action_complete rclcpp rclcpp_action ros2lings_interfaces)
endif()

ament_package()
