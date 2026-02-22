cmake_minimum_required(VERSION 3.8)
project(ros2lings_34_conditional_launch)

find_package(ament_cmake REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/sim_node.py
  src/real_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_conditional_launch test/test_conditional_launch.py
    TIMEOUT 30
  )
endif()

ament_package()
