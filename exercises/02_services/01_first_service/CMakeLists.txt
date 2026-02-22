cmake_minimum_required(VERSION 3.8)
project(ros2lings_16_first_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2lings_interfaces REQUIRED)

add_executable(first_service src/first_service.cpp)
ament_target_dependencies(first_service rclcpp ros2lings_interfaces)
install(TARGETS first_service DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_first_service test/test_first_service.cpp)
  target_compile_definitions(test_first_service PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_first_service rclcpp ros2lings_interfaces)
endif()

ament_package()
