cmake_minimum_required(VERSION 3.16)
project(point_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor number:  ${N}")

  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message(STATUS "core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message(STATUS "core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(OpenMP QUIET)
find_package(PythonLibs REQUIRED) # Consider using Pybind11 for ROS2
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

# ZLIO's pure C++ multi-frame initialization core (GPL-2.0-only).
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp REQUIRED)
set(ZLIO_STARTUP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/zlio_startup)
set(ZLIO_STARTUP_SOURCES config execution geometry initialization plane preintegration
  registration state uncertainty window inertial trajectory input)
list(TRANSFORM ZLIO_STARTUP_SOURCES PREPEND "${ZLIO_STARTUP_DIR}/src/")
list(TRANSFORM ZLIO_STARTUP_SOURCES APPEND ".cpp")
add_library(pointlio_joint_startup STATIC ${ZLIO_STARTUP_SOURCES} src/joint_startup.cpp)
target_include_directories(pointlio_joint_startup PUBLIC include ${ZLIO_STARTUP_DIR}/include
  PRIVATE ${NANOFLANN_INCLUDE_DIR})
target_link_libraries(pointlio_joint_startup PUBLIC Eigen3::Eigen yaml-cpp Threads::Threads)

# Declare a ROS2 executable
add_executable(pointlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
)

ament_target_dependencies(pointlio_mapping
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  diagnostic_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

target_link_libraries(pointlio_mapping pointlio_joint_startup ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_adaptive_lio test/test_adaptive_lio.cpp)
  target_include_directories(test_adaptive_lio PRIVATE include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_adaptive_lio rclcpp nav_msgs sensor_msgs tf2_ros)
  target_link_libraries(test_adaptive_lio ${PCL_LIBRARIES})
  ament_add_gtest(test_joint_startup test/test_joint_startup.cpp TIMEOUT 180)
  target_link_libraries(test_joint_startup pointlio_joint_startup ${PCL_LIBRARIES})
  ament_target_dependencies(test_joint_startup rclcpp nav_msgs sensor_msgs tf2_ros)
  add_executable(benchmark_startup test/benchmark_startup.cpp src/IMU_Processing.cpp)
  target_include_directories(benchmark_startup PRIVATE src)
  target_link_libraries(benchmark_startup pointlio_joint_startup ${PCL_LIBRARIES})
  ament_target_dependencies(benchmark_startup rclcpp nav_msgs sensor_msgs tf2_ros pcl_conversions)
  ament_add_gtest(test_imu_update test/test_imu_update.cpp)
  target_include_directories(test_imu_update PRIVATE include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_imu_update rclcpp nav_msgs sensor_msgs tf2_ros)
  target_link_libraries(test_imu_update ${PCL_LIBRARIES})
endif()

# Install the executable
install(TARGETS
  pointlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
)

ament_package()

install(DIRECTORY third_party/zlio_startup DESTINATION share/${PROJECT_NAME}/third_party)
