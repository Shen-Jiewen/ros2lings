cmake_minimum_required(VERSION 3.8)
project(ros2lings_36_launch_composition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# 编译 publisher 组件为共享库
add_library(publisher_component SHARED src/publisher_component.cpp)
ament_target_dependencies(publisher_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(publisher_component "PublisherComponent")

# 编译 subscriber 组件为共享库
add_library(subscriber_component SHARED src/subscriber_component.cpp)
ament_target_dependencies(subscriber_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(subscriber_component "SubscriberComponent")

# 安装共享库
install(TARGETS publisher_component subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# 安装 Launch 文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_composition test/test_launch_composition.py
    TIMEOUT 30
  )
endif()

ament_package()
