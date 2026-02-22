cmake_minimum_required(VERSION 3.8)
project(ros2lings_37_launch_events)

find_package(ament_cmake REQUIRED)

# 安装 Python 源文件（探索模式问答）
install(PROGRAMS
  src/launch_events_quiz.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_events test/test_launch_events.py
    TIMEOUT 30
  )
endif()

ament_package()
