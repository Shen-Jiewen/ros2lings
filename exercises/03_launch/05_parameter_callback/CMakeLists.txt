cmake_minimum_required(VERSION 3.8)
project(ros2lings_32_parameter_callback)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(parameter_callback src/parameter_callback.cpp)
ament_target_dependencies(parameter_callback rclcpp rcl_interfaces)
install(TARGETS parameter_callback DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_parameter_callback test/test_parameter_callback.cpp)
  target_compile_definitions(test_parameter_callback PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_parameter_callback rclcpp rcl_interfaces)
endif()

ament_package()
