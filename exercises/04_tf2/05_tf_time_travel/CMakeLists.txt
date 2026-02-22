cmake_minimum_required(VERSION 3.8)
project(ros2lings_42_tf_time_travel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_time_travel src/tf_time_travel.cpp)
ament_target_dependencies(tf_time_travel rclcpp tf2_ros tf2 geometry_msgs)
install(TARGETS tf_time_travel DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf_time_travel test/test_tf_time_travel.cpp)
  ament_target_dependencies(test_tf_time_travel rclcpp tf2_ros tf2 geometry_msgs)
endif()

ament_package()
