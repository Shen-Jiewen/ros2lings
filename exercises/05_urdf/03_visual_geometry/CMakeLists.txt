cmake_minimum_required(VERSION 3.8)
project(ros2lings_50_visual_geometry)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_visual_geometry test/test_visual_geometry.py TIMEOUT 30)
endif()

ament_package()
