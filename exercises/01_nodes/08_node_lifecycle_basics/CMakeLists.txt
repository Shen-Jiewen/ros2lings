cmake_minimum_required(VERSION 3.8)
project(ros2lings_08_node_lifecycle_basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(node_lifecycle_basics src/node_lifecycle_basics.cpp)
ament_target_dependencies(node_lifecycle_basics rclcpp)
install(TARGETS node_lifecycle_basics DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_library(node_lifecycle_basics_lib src/node_lifecycle_basics.cpp)
  ament_target_dependencies(node_lifecycle_basics_lib rclcpp)
  target_compile_definitions(node_lifecycle_basics_lib PRIVATE ROS2LINGS_TEST)

  ament_add_gtest(test_node_lifecycle_basics test/test_node_lifecycle_basics.cpp)
  ament_target_dependencies(test_node_lifecycle_basics rclcpp)
  target_link_libraries(test_node_lifecycle_basics node_lifecycle_basics_lib)
endif()

ament_package()
