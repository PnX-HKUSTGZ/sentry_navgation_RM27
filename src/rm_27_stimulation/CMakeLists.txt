cmake_minimum_required(VERSION 3.8)
project(rm_27_stimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)
find_package(Protobuf REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(ros2_livox SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
)

target_include_directories(ros2_livox PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
)

ament_target_dependencies(ros2_livox
  rclcpp
  gazebo_ros
  sensor_msgs
  livox_ros_driver2
)

target_link_libraries(ros2_livox
  ${GAZEBO_LIBRARIES}
  RayPlugin
  GpuRayPlugin
  Boost::chrono
  protobuf::libprotobuf
)

install(TARGETS ros2_livox
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  meshes
  rviz
  scan_mode
  urdf
  world
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
