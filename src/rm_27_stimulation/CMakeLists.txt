cmake_minimum_required(VERSION 3.8)
project(rm_27_stimulation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(rm27_cmd_vel_pose_control_system SHARED
  src/cmd_vel_pose_control_system.cpp
)
target_include_directories(rm27_cmd_vel_pose_control_system PRIVATE include)
target_link_libraries(rm27_cmd_vel_pose_control_system
  gz-msgs10::gz-msgs10
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

add_executable(rm27_ground_truth_localizer
  src/ground_truth_localizer.cpp
)
target_include_directories(rm27_ground_truth_localizer PRIVATE include)
ament_target_dependencies(rm27_ground_truth_localizer
  geometry_msgs
  nav_msgs
  pcl_ros
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ground_truth_twist_transform
    test/test_ground_truth_twist_transform.cpp
  )
  if(TARGET test_ground_truth_twist_transform)
    target_include_directories(test_ground_truth_twist_transform PRIVATE include)
    ament_target_dependencies(test_ground_truth_twist_transform
      geometry_msgs
      tf2
    )
  endif()

  ament_add_gtest(test_planar_velocity_pi
    test/test_planar_velocity_pi.cpp
  )
  if(TARGET test_planar_velocity_pi)
    target_include_directories(test_planar_velocity_pi PRIVATE include)
  endif()

  ament_add_gtest(test_stiction_assist
    test/test_stiction_assist.cpp
  )
  if(TARGET test_stiction_assist)
    target_include_directories(test_stiction_assist PRIVATE include)
  endif()

  ament_add_gtest(test_organized_lidar_miss_rays
    test/test_organized_lidar_miss_rays.cpp
  )
  if(TARGET test_organized_lidar_miss_rays)
    target_include_directories(test_organized_lidar_miss_rays PRIVATE include)
    ament_target_dependencies(test_organized_lidar_miss_rays sensor_msgs)
  endif()
endif()

install(TARGETS
  rm27_cmd_vel_pose_control_system
  rm27_ground_truth_localizer
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

install(DIRECTORY include/rm_27_stimulation
  DESTINATION include
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "obstacle_plugin" EXCLUDE
)

ament_package()
