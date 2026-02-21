cmake_minimum_required(VERSION 3.8)
project(ros2lings_27_action_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2lings_interfaces REQUIRED)

add_executable(action_state_machine src/action_state_machine.cpp)
ament_target_dependencies(action_state_machine rclcpp rclcpp_action ros2lings_interfaces)
install(TARGETS action_state_machine DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_state_machine test/test_action_state_machine.cpp)
  ament_target_dependencies(test_action_state_machine rclcpp rclcpp_action ros2lings_interfaces)
endif()

ament_package()
