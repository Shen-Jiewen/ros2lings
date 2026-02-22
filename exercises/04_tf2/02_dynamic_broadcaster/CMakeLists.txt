cmake_minimum_required(VERSION 3.8)
project(ros2lings_39_dynamic_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dynamic_broadcaster src/dynamic_broadcaster.cpp)
ament_target_dependencies(dynamic_broadcaster rclcpp tf2_ros geometry_msgs)
install(TARGETS dynamic_broadcaster DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamic_broadcaster test/test_dynamic_broadcaster.cpp)
  ament_target_dependencies(test_dynamic_broadcaster rclcpp tf2_ros geometry_msgs)
endif()

ament_package()
