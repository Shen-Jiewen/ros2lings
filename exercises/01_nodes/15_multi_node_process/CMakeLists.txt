cmake_minimum_required(VERSION 3.8)
project(ros2lings_15_multi_node_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(multi_node_process src/multi_node_process.cpp)
ament_target_dependencies(multi_node_process rclcpp std_msgs)
install(TARGETS multi_node_process DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_multi_node_process test/test_multi_node_process.cpp)
  ament_target_dependencies(test_multi_node_process rclcpp std_msgs)
endif()

ament_package()
