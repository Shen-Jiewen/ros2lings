cmake_minimum_required(VERSION 3.8)
project(ros2lings_12_qos_mismatch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(qos_mismatch src/qos_mismatch.cpp)
ament_target_dependencies(qos_mismatch rclcpp std_msgs)
install(TARGETS qos_mismatch DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_qos_mismatch test/test_qos_mismatch.cpp)
  target_compile_definitions(test_qos_mismatch PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_qos_mismatch rclcpp std_msgs)
endif()

ament_package()
