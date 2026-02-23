cmake_minimum_required(VERSION 3.8)
project(ros2lings_05_custom_message)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
)

add_executable(custom_message src/custom_message.cpp)
ament_target_dependencies(custom_message rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(custom_message "${cpp_typesupport_target}")
install(TARGETS custom_message DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_custom_message test/test_custom_message.cpp)
  target_compile_definitions(test_custom_message PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_custom_message rclcpp)
  target_link_libraries(test_custom_message "${cpp_typesupport_target}")
endif()

ament_package()
