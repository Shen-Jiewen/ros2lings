cmake_minimum_required(VERSION 3.8)
project(ros2lings_24_custom_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Countdown.action"
)

add_executable(custom_action src/custom_action.cpp)
ament_target_dependencies(custom_action rclcpp rclcpp_action)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(custom_action "${typesupport_target}")
install(TARGETS custom_action DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_custom_action test/test_custom_action.cpp)
  target_compile_definitions(test_custom_action PRIVATE ROS2LINGS_TEST)
  ament_target_dependencies(test_custom_action rclcpp rclcpp_action)
  target_link_libraries(test_custom_action "${typesupport_target}")
endif()

ament_package()
