cmake_minimum_required(VERSION 3.8)
project(ros2lings_13_component_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# 组件编译为共享库
add_library(talker_component SHARED src/component_node.cpp)
ament_target_dependencies(talker_component rclcpp rclcpp_components std_msgs)

# 注册组件，使其可以被 component_container 加载
rclcpp_components_register_nodes(talker_component "TalkerComponent")

install(TARGETS talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_component_node test/test_component_node.cpp)
  ament_target_dependencies(test_component_node rclcpp std_msgs)
endif()

ament_package()
