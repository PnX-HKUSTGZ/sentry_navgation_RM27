cmake_minimum_required(VERSION 3.11)
project(relocalization_manager)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
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
find_package(PCL REQUIRED COMPONENTS common io filters features kdtree search)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FLANN REQUIRED IMPORTED_TARGET flann)
ament_auto_find_build_dependencies()

# Add small_gicp dependency
include(FetchContent)
fetchcontent_declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp.git
  GIT_TAG "master"
)
fetchcontent_makeavailable(small_gicp)

# TEASER++ is a pure CMake dependency. Keep the dependency graph pinned so ROS 2 Jazzy builds are
# reproducible and do not pull the ROS 1 catkin fork used by the reference project.
fetchcontent_declare(
  pmc
  GIT_REPOSITORY https://github.com/jingnanshi/pmc.git
  GIT_TAG a2dfd612a501bca83c47206255dbbff619481f97
)
fetchcontent_declare(
  tinyply
  GIT_REPOSITORY https://github.com/ddiakopoulos/tinyply.git
  GIT_TAG c9bb690dfe5e9105961e9e28120c48c9ae084bc6
)
set(RELOCALIZATION_MANAGER_BUILD_TESTING ${BUILD_TESTING})
set(BUILD_TESTING OFF CACHE BOOL "Disable dependency tests" FORCE)
set(BUILD_DOC OFF CACHE BOOL "Disable TEASER++ documentation" FORCE)
set(BUILD_TEASER_FPFH OFF CACHE BOOL "Use the package PCL FPFH implementation" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Disable TEASER++ Python bindings" FORCE)
set(BUILD_MATLAB_BINDINGS OFF CACHE BOOL "Disable TEASER++ MATLAB bindings" FORCE)
set(ENABLE_DIAGNOSTIC_PRINT OFF CACHE BOOL "Disable TEASER++ diagnostic stdout" FORCE)
fetchcontent_declare(
  teaserpp
  GIT_REPOSITORY https://github.com/MIT-SPARK/TEASER-plusplus.git
  GIT_TAG 52a9c52ee7d4c838c5e8a75458c33178be5bfb70
)
fetchcontent_makeavailable(teaserpp)
set(BUILD_TESTING ${RELOCALIZATION_MANAGER_BUILD_TESTING} CACHE BOOL "Build package tests" FORCE)

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_directories(${PROJECT_NAME} PRIVATE ${FLANN_LIBRARY_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  small_gicp
  teaserpp::teaser_registration
  PkgConfig::FLANN
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN relocalization_manager::RelocalizationManagerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
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
