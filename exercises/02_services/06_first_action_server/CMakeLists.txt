cmake_minimum_required(VERSION 3.8)
project(ros2lings_21_first_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2lings_interfaces REQUIRED)

add_executable(first_action_server src/first_action_server.cpp)
ament_target_dependencies(first_action_server rclcpp rclcpp_action ros2lings_interfaces)
install(TARGETS first_action_server DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_first_action_server test/test_first_action_server.cpp src/first_action_server.cpp)
  target_compile_definitions(test_first_action_server PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_first_action_server rclcpp rclcpp_action ros2lings_interfaces)
endif()

ament_package()
