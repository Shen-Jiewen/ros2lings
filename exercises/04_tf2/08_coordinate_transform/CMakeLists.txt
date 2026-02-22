cmake_minimum_required(VERSION 3.8)
project(ros2lings_45_coordinate_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(coordinate_transform src/coordinate_transform.cpp)
ament_target_dependencies(coordinate_transform rclcpp tf2_ros tf2 tf2_geometry_msgs geometry_msgs)
install(TARGETS coordinate_transform DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinate_transform test/test_coordinate_transform.cpp TIMEOUT 120)
  target_compile_definitions(test_coordinate_transform PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_coordinate_transform rclcpp tf2_ros tf2 tf2_geometry_msgs geometry_msgs)
endif()

ament_package()
