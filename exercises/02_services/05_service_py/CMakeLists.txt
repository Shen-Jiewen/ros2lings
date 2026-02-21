cmake_minimum_required(VERSION 3.8)
project(ros2lings_20_service_py)

find_package(ament_cmake REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/service_py.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_service_py test/test_service_py.py
    TIMEOUT 30
  )
endif()

ament_package()
