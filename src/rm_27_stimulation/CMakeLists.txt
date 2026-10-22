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

add_library(rm27_ground_truth_odometry_system SHARED
  src/ground_truth_odometry_system.cpp
)

add_library(rm27_pose_imu_system SHARED src/pose_imu_system.cpp)
target_include_directories(rm27_pose_imu_system PRIVATE include)
target_link_libraries(rm27_pose_imu_system
  gz-msgs10::gz-msgs10 gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8 gz-transport13::gz-transport13)
target_include_directories(rm27_ground_truth_odometry_system PRIVATE include)
target_link_libraries(rm27_ground_truth_odometry_system
  gz-msgs10::gz-msgs10 gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8 gz-transport13::gz-transport13
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
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_pose_imu test/test_pose_imu.cpp)
  if(TARGET test_pose_imu)
    target_include_directories(test_pose_imu PRIVATE include)
    target_link_libraries(test_pose_imu gz-msgs10::gz-msgs10 gz-sim8::gz-sim8)
  endif()
  add_test(NAME test_gimbal_state
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gimbal_state.py)
  add_test(NAME test_simulation_startup
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_startup.py)

  ament_add_gtest(test_ground_truth_odometry test/test_ground_truth_odometry.cpp)
  if(TARGET test_ground_truth_odometry)
    target_include_directories(test_ground_truth_odometry PRIVATE include)
    target_link_libraries(test_ground_truth_odometry
      gz-msgs10::gz-msgs10 gz-sim8::gz-sim8)
  endif()

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
  rm27_ground_truth_odometry_system
  rm27_pose_imu_system
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

install(PROGRAMS scripts/gimbal_control.py scripts/gimbal_panel.py scripts/wait_for_sim_clock.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/gimbal_state.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "obstacle_plugin" EXCLUDE
)

ament_package()
