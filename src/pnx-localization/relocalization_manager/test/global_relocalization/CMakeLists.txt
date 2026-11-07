foreach(name bounded_kdtree core visualization raw_frontend raw_scan_worker competition_state)
  ament_add_gtest(test_${name} test_${name}.cpp)
  target_include_directories(test_${name} PRIVATE ../../include)
  target_link_libraries(test_${name} global_relocalization_core)
endforeach()
ament_target_dependencies(test_visualization rclcpp geometry_msgs sensor_msgs std_msgs
  visualization_msgs pcl_conversions)
ament_add_pytest_test(test_action_contract test_action_contract.py
  TIMEOUT 90 ENV "GLOBAL_RELOCALIZATION_NODE=$<TARGET_FILE:global_relocalization_node>")
ament_add_pytest_test(test_competition_contract test_competition_contract.py
  TIMEOUT 45 ENV "GLOBAL_RELOCALIZATION_NODE=$<TARGET_FILE:global_relocalization_node>")
ament_add_pytest_test(test_startup_handoff test_startup_handoff.py
  TIMEOUT 45 ENV "RELOCALIZATION_TEST_EXECUTABLE=$<TARGET_FILE:relocalization_manager_node>")
