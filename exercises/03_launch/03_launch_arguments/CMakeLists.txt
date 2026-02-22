cmake_minimum_required(VERSION 3.8)
project(ros2lings_30_launch_arguments)

find_package(ament_cmake REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/configurable_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_arguments test/test_launch_arguments.py
    TIMEOUT 30
  )
endif()

ament_package()
