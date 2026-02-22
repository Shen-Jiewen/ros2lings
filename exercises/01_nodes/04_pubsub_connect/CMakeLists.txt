cmake_minimum_required(VERSION 3.8)
project(ros2lings_04_pubsub_connect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pubsub_connect src/pubsub_connect.cpp)
ament_target_dependencies(pubsub_connect rclcpp std_msgs)
install(TARGETS pubsub_connect DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pubsub_connect test/test_pubsub_connect.cpp)
  target_compile_definitions(test_pubsub_connect PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_pubsub_connect rclcpp std_msgs)
endif()

ament_package()
