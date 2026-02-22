cmake_minimum_required(VERSION 3.8)
project(ros2lings_18_service_pair)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2lings_interfaces REQUIRED)

add_executable(service_pair src/service_pair.cpp)
ament_target_dependencies(service_pair rclcpp ros2lings_interfaces)
install(TARGETS service_pair DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_service_pair test/test_service_pair.cpp)
  target_compile_definitions(test_service_pair PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_service_pair rclcpp ros2lings_interfaces)
endif()

ament_package()
