cmake_minimum_required(VERSION 3.8)
project(minco_planner)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros_interfaces REQUIRED)
find_package(rog_map REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

nav2_package()

set(library_name minco_planner)

file(GLOB UTILS_SOURCES "src/utils/*.cpp")
file(GLOB SMAC_SEARCH_SOURCES "src/smac_search/*.cpp")
set(MINCO_CORE_SOURCES
  src/minco_core/astar.cpp
  src/minco_core/corridor_generator.cpp
  src/minco_core/components/global_path_searcher.cpp
  src/minco_core/components/local_path_processor.cpp
  src/minco_core/components/map_query_adapters.cpp
  src/minco_core/components/planner_mode_context.cpp
  src/minco_core/components/recovery_behaivor.cpp
  src/minco_core/components/trajectory_safety_checker.cpp
  src/minco_core/minco_fsm.cpp
  src/minco_core/minco_planner.cpp
  src/minco_core/minco_utils.cpp
  src/minco_core/visualizer.cpp
  src/performance/planner_performance_monitor.cpp
)
file(GLOB TRAJ_OPT_SOURCES "src/traj_opt/*.cpp")

add_library(${library_name} SHARED
  ${UTILS_SOURCES}
  ${SMAC_SEARCH_SOURCES}
  ${MINCO_CORE_SOURCES}
  ${TRAJ_OPT_SOURCES}
)

target_compile_definitions(${library_name}
PUBLIC
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
  ORIGIN_AT_CORNER
  USE_ROS2
)

target_compile_options(${library_name}
PRIVATE
  # nav2_package() enables -Werror globally. Keep its diagnostics visible, but
  # do not let warnings from imported QuickHull/Eigen code block integration.
  -Wno-error
  -Wno-error=unused-parameter
  -Wno-error=unused-function
  -Wno-error=ignored-qualifiers
)

target_include_directories(${library_name}
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${library_name}
  builtin_interfaces
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  ros_interfaces
  rog_map
)

target_link_libraries(${library_name}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  yaml-cpp
)

# The stock planner_server executable uses a single-threaded executor. Active
# MINCO embeds ROG-map in the planner plugin, so its sensor/update callback
# groups must remain schedulable while planning and trajectory safety checks
# are running.
add_executable(planner_server_mt src/planner_server_mt.cpp)
ament_target_dependencies(planner_server_mt
  nav2_planner
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_safety_checker
    test/trajectory_safety_checker_test.cpp
  )
  target_compile_options(test_trajectory_safety_checker PRIVATE -Wno-error)
  target_link_libraries(test_trajectory_safety_checker ${library_name})
  ament_target_dependencies(test_trajectory_safety_checker
    nav2_costmap_2d
    rclcpp
    rog_map
  )

  ament_add_gtest(test_local_path_processor
    test/local_path_processor_test.cpp
  )
  target_compile_options(test_local_path_processor PRIVATE -Wno-error)
  target_link_libraries(test_local_path_processor ${library_name})
  ament_target_dependencies(test_local_path_processor
    nav2_costmap_2d
    rclcpp
    rog_map
  )

  ament_add_gtest(test_planning_session_state
    test/planning_session_state_test.cpp
  )
  target_include_directories(test_planning_session_state PRIVATE include)

  ament_add_gtest(test_planning_request_contract
    test/planning_request_contract_test.cpp
  )
  target_include_directories(test_planning_request_contract PRIVATE include)
  ament_target_dependencies(test_planning_request_contract
    builtin_interfaces
    geometry_msgs
  )

  ament_add_gtest(test_planning_request_lease
    test/planning_request_lease_test.cpp
  )
  target_include_directories(test_planning_request_lease PRIVATE include)
  ament_target_dependencies(test_planning_request_lease
    builtin_interfaces
    geometry_msgs
  )

  ament_add_gtest(test_cached_trajectory_policy
    test/cached_trajectory_policy_test.cpp
  )
  target_include_directories(test_cached_trajectory_policy PRIVATE include)

  ament_add_gtest(test_map_query_adapters
    test/map_query_adapters_test.cpp
  )
  target_compile_options(test_map_query_adapters PRIVATE -Wno-error)
  target_link_libraries(test_map_query_adapters ${library_name})
  ament_target_dependencies(test_map_query_adapters
    nav2_costmap_2d
    rog_map
  )

  ament_add_gtest(test_smac_global_search_policy
    test/smac_global_search_policy_test.cpp
  )
  target_compile_options(test_smac_global_search_policy PRIVATE -Wno-error)
  target_link_libraries(test_smac_global_search_policy ${library_name})
  ament_target_dependencies(test_smac_global_search_policy
    nav2_costmap_2d
    rog_map
  )

  ament_add_gtest(test_minco_optimizer_dynamics
    test/minco_optimizer_dynamics_test.cpp
  )
  target_compile_options(test_minco_optimizer_dynamics PRIVATE -Wno-error)
  target_link_libraries(test_minco_optimizer_dynamics ${library_name})

  ament_add_gtest(test_yaw_traj_opt
    test/yaw_traj_opt_test.cpp
  )
  target_compile_options(test_yaw_traj_opt PRIVATE -Wno-error)
  target_link_libraries(test_yaw_traj_opt ${library_name})
endif()

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS planner_server_mt
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_planner
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  ros_interfaces
  rog_map
)
ament_export_targets(${library_name})
ament_package()
