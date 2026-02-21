cmake_minimum_required(VERSION 3.8)
project(ros2lings_07_topic_introspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(topic_introspection src/topic_introspection.cpp)
ament_target_dependencies(topic_introspection rclcpp std_msgs)
install(TARGETS topic_introspection DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 将源文件编译为库供测试使用
  add_library(topic_introspection_lib src/topic_introspection.cpp)
  ament_target_dependencies(topic_introspection_lib rclcpp std_msgs)
  set_target_properties(topic_introspection_lib PROPERTIES COMPILE_DEFINITIONS "TESTING_BUILD")

  ament_add_gtest(test_topic_introspection test/test_topic_introspection.cpp)
  ament_target_dependencies(test_topic_introspection rclcpp std_msgs)
  target_link_libraries(test_topic_introspection topic_introspection_lib)
endif()

ament_package()
