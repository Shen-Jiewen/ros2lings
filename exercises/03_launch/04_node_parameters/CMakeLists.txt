cmake_minimum_required(VERSION 3.8)
project(ros2lings_31_node_parameters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(node_parameters src/node_parameters.cpp)
ament_target_dependencies(node_parameters rclcpp)
install(TARGETS node_parameters DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_node_parameters test/test_node_parameters.cpp)
  target_compile_definitions(test_node_parameters PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_node_parameters rclcpp)
endif()

ament_package()
