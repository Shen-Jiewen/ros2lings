cmake_minimum_required(VERSION 3.8)
project(ros2lings_47_multi_robot_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(multi_robot_tf src/multi_robot_tf.cpp)
ament_target_dependencies(multi_robot_tf rclcpp tf2_ros geometry_msgs)
install(TARGETS multi_robot_tf DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_multi_robot_tf test/test_multi_robot_tf.cpp)
  ament_target_dependencies(test_multi_robot_tf rclcpp tf2_ros geometry_msgs)
endif()

ament_package()
