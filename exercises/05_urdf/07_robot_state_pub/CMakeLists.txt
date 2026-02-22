cmake_minimum_required(VERSION 3.8)
project(ros2lings_54_robot_state_pub)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_state_pub test/test_robot_state_pub.py TIMEOUT 30)
endif()

ament_package()
