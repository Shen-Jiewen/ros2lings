cmake_minimum_required(VERSION 3.8)
project(ros2lings_02_first_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(first_publisher src/first_publisher.cpp)
ament_target_dependencies(first_publisher rclcpp std_msgs)
install(TARGETS first_publisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_first_publisher test/test_first_publisher.cpp src/first_publisher.cpp)
  target_compile_definitions(test_first_publisher PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_first_publisher rclcpp std_msgs)
endif()

ament_package()
