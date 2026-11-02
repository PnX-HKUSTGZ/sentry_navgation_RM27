cmake_minimum_required(VERSION 3.8)
project(rm27_rendering)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rendering test/test_rendering.py TIMEOUT 30)
endif()

ament_package()
