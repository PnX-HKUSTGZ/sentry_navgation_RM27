cmake_minimum_required(VERSION 3.8)
project(pb2025_nav_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(minco_shadow_goal_relay_bt_node SHARED
  src/minco_shadow_goal_relay.cpp
)
target_compile_definitions(minco_shadow_goal_relay_bt_node PRIVATE BT_PLUGIN_EXPORT)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_minco_shadow_goal_relay
    test/test_minco_shadow_goal_relay.cpp
  )
  target_link_libraries(test_minco_shadow_goal_relay
    minco_shadow_goal_relay_bt_node
  )
  ament_target_dependencies(test_minco_shadow_goal_relay
    behaviortree_cpp
    geometry_msgs
    lifecycle_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
  )

  ament_add_pytest_test(test_navigation_launch
    test/test_navigation_launch.py
    TIMEOUT 30
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  pcd
  config
  behavior_trees
  rviz
)
