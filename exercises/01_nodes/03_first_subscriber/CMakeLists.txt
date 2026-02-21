cmake_minimum_required(VERSION 3.8)
project(ros2lings_03_first_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(first_subscriber src/first_subscriber.cpp)
ament_target_dependencies(first_subscriber rclcpp std_msgs)
install(TARGETS first_subscriber DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_first_subscriber test/test_first_subscriber.cpp)
  ament_target_dependencies(test_first_subscriber rclcpp std_msgs)
endif()

ament_package()
