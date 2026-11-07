cmake_minimum_required(VERSION 3.16)
project(relocalization_manager)

## Local tracking and the integrated global core share C++17 and one small_gicp revision.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(PkgConfig REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(FLANN REQUIRED IMPORTED_TARGET flann)
ament_auto_find_build_dependencies()

# Add small_gicp dependency
include(FetchContent)
fetchcontent_declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp.git
  GIT_TAG fa0cfc983417c044b6036f71dfa448038bb9c820
)
fetchcontent_makeavailable(small_gicp)
set_target_properties(small_gicp PROPERTIES EXPORT_COMPILE_COMMANDS OFF)

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/relocalization_manager.cpp
)

target_link_directories(${PROJECT_NAME} PRIVATE ${FLANN_LIBRARY_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  small_gicp
  PkgConfig::FLANN
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN relocalization_manager::RelocalizationManagerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# Global retrieval remains a separate executor so its bounded raw workers cannot
# block the sole map->odom publisher. Both executables belong to this package.
add_library(global_relocalization_core SHARED
  src/global_relocalization/core.cpp src/global_relocalization/stationary_raw_frontend.cpp)
target_include_directories(global_relocalization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_directories(global_relocalization_core PUBLIC ${FLANN_LIBRARY_DIRS})
target_link_libraries(global_relocalization_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX small_gicp PkgConfig::FLANN OpenSSL::Crypto nlohmann_json::nlohmann_json)
add_executable(global_relocalization_node src/global_relocalization/node.cpp)
target_link_libraries(global_relocalization_node global_relocalization_core)
ament_target_dependencies(global_relocalization_node rclcpp rclcpp_action
  global_relocalization_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
  visualization_msgs tf2 tf2_ros tf2_eigen pcl_conversions)
add_executable(global_relocalization_offline_query src/global_relocalization/offline_query.cpp)
target_link_libraries(global_relocalization_offline_query global_relocalization_core yaml-cpp)
add_executable(global_relocalization_raw_replay src/global_relocalization/raw_replay.cpp)
target_link_libraries(global_relocalization_raw_replay global_relocalization_core)
install(TARGETS global_relocalization_core global_relocalization_node
  global_relocalization_offline_query global_relocalization_raw_replay
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/display_helper.py scripts/relocalize_then_navigate.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES ${small_gicp_SOURCE_DIR}/LICENSE
  DESTINATION share/${PROJECT_NAME}/third_party RENAME small_gicp_LICENSE)
ament_python_install_package(relocalization_manager_launch)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gicp_verification test/test_gicp_verification.cpp)
  target_link_libraries(test_gicp_verification ${PROJECT_NAME} small_gicp ${PCL_LIBRARIES})
  target_include_directories(test_gicp_verification PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_directories(test_gicp_verification PRIVATE ${FLANN_LIBRARY_DIRS})
  ament_target_dependencies(test_gicp_verification rclcpp sensor_msgs geometry_msgs tf2_ros)

  ament_add_gtest(test_stationary_gate test/test_stationary_gate.cpp)
  target_include_directories(test_stationary_gate PRIVATE include)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test/global_relocalization)
  ament_add_pytest_test(test_stationary_relocalization test/test_stationary_relocalization.py
    TIMEOUT 90
    ENV RELOCALIZATION_TEST_EXECUTABLE=$<TARGET_FILE:relocalization_manager_node>)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  rviz
)
