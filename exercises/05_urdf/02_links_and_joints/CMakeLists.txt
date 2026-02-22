cmake_minimum_required(VERSION 3.8)
project(ros2lings_49_links_and_joints)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_links_and_joints test/test_links_and_joints.py TIMEOUT 30)
endif()

ament_package()
