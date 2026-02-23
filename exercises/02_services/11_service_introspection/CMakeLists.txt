cmake_minimum_required(VERSION 3.8)
project(ros2lings_26_service_introspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(service_introspection src/service_introspection.cpp)
ament_target_dependencies(service_introspection rclcpp)
install(TARGETS service_introspection DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 将源文件编译为库供测试使用
  add_library(service_introspection_lib src/service_introspection.cpp)
  ament_target_dependencies(service_introspection_lib rclcpp)
  target_compile_definitions(service_introspection_lib PRIVATE ROS2LINGS_TEST)

  ament_add_gtest(test_service_introspection test/test_service_introspection.cpp)
  ament_target_dependencies(test_service_introspection rclcpp)
  target_link_libraries(test_service_introspection service_introspection_lib)
endif()

ament_package()
