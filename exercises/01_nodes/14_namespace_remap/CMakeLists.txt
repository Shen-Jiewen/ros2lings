cmake_minimum_required(VERSION 3.8)
project(ros2lings_14_namespace_remap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(namespace_remap src/namespace_remap.cpp)
ament_target_dependencies(namespace_remap rclcpp std_msgs)
install(TARGETS namespace_remap DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_namespace_remap test/test_namespace_remap.cpp)
  ament_target_dependencies(test_namespace_remap rclcpp std_msgs)
endif()

ament_package()
