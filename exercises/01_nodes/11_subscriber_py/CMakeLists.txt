cmake_minimum_required(VERSION 3.8)
project(ros2lings_11_subscriber_py)

find_package(ament_cmake REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/subscriber_py.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_subscriber_py test/test_subscriber_py.py
    TIMEOUT 30
  )
endif()

ament_package()
