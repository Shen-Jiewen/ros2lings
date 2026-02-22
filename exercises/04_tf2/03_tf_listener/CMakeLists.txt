cmake_minimum_required(VERSION 3.8)
project(ros2lings_40_tf_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener rclcpp tf2_ros tf2 geometry_msgs)
install(TARGETS tf_listener DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf_listener test/test_tf_listener.cpp TIMEOUT 120)
  target_compile_definitions(test_tf_listener PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_tf_listener rclcpp tf2_ros tf2 geometry_msgs)
endif()

ament_package()
