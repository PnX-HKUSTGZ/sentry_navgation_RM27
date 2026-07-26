cmake_minimum_required(VERSION 3.8)
project(rm_27_stimulation)

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(rm27_cmd_vel_pose_control_system SHARED
  src/cmd_vel_pose_control_system.cpp
)
target_link_libraries(rm27_cmd_vel_pose_control_system
  gz-msgs10::gz-msgs10
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

install(TARGETS rm27_cmd_vel_pose_control_system
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  config
  launch
  rviz
  scan_mode
  urdf
  world
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "obstacle_plugin" EXCLUDE
)

ament_package()
