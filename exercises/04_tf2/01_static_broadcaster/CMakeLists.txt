cmake_minimum_required(VERSION 3.8)
project(ros2lings_38_static_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(static_broadcaster src/static_broadcaster.cpp)
ament_target_dependencies(static_broadcaster rclcpp tf2_ros geometry_msgs)
install(TARGETS static_broadcaster DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_static_broadcaster test/test_static_broadcaster.cpp)
  ament_target_dependencies(test_static_broadcaster rclcpp tf2_ros geometry_msgs)
endif()

ament_package()
