cmake_minimum_required(VERSION 3.8)
project(ros2lings_01_hello_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp)
install(TARGETS hello_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hello_node test/test_hello_node.cpp)
  target_compile_definitions(test_hello_node PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_hello_node rclcpp)
endif()

ament_package()
