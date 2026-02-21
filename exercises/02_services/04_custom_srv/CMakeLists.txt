cmake_minimum_required(VERSION 3.8)
project(ros2lings_19_custom_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeArea.srv"
)

add_executable(custom_srv src/custom_srv.cpp)
ament_target_dependencies(custom_srv rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(custom_srv "${cpp_typesupport_target}")
install(TARGETS custom_srv DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_custom_srv test/test_custom_srv.cpp)
  ament_target_dependencies(test_custom_srv rclcpp)
  target_link_libraries(test_custom_srv "${cpp_typesupport_target}")
endif()

ament_package()
