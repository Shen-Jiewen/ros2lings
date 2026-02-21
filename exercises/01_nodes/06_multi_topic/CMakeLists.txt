cmake_minimum_required(VERSION 3.8)
project(ros2lings_06_multi_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(multi_topic src/multi_topic.cpp)
ament_target_dependencies(multi_topic rclcpp std_msgs)
install(TARGETS multi_topic DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_multi_topic test/test_multi_topic.cpp)
  ament_target_dependencies(test_multi_topic rclcpp std_msgs)
endif()

ament_package()
