cmake_minimum_required(VERSION 3.11)
project(relocalization_manager)

## 公共几何与恢复接口使用 C++17
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

# 无 ROS 状态和 TF 写入权的公共几何与恢复计算库，由后端在同一进程调用。
find_package(TBB REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(rm27_relocalization SHARED src/measurement.cpp src/scan_context.cpp
  src/map_matcher.cpp src/recovery_engine.cpp)
target_include_directories(rm27_relocalization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_directories(rm27_relocalization PUBLIC ${FLANN_LIBRARY_DIRS})
target_link_libraries(rm27_relocalization PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} TBB::tbb yaml-cpp PRIVATE small_gicp OpenSSL::Crypto)
add_executable(recovery_probe src/recovery_probe.cpp)
target_link_libraries(recovery_probe rm27_relocalization)
install(TARGETS recovery_probe DESTINATION lib/${PROJECT_NAME})
install(TARGETS rm27_relocalization EXPORT rm27_relocalization_targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(rm27_relocalization_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL TBB yaml-cpp)

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
)
