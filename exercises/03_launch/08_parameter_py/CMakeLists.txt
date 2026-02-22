cmake_minimum_required(VERSION 3.8)
project(ros2lings_35_parameter_py)

find_package(ament_cmake REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/parameter_py.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_parameter_py test/test_parameter_py.py
    TIMEOUT 60
  )
endif()

ament_package()
