cmake_minimum_required(VERSION 3.10)
project(minco_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

option(MINCO_DEBUG "Enable Minco debug printing" OFF)

find_package(qpOASES QUIET)
if(NOT qpOASES_FOUND)
  find_path(QPOASES_INCLUDE_DIR qpOASES.hpp PATH_SUFFIXES qpOASES qpoases)
  find_library(QPOASES_LIBRARY qpOASES)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/input_validation.cpp
  src/mpc_solver.cpp
  src/minco_mpc_controller.cpp
  src/performance/mpc_performance_monitor.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

if(MINCO_DEBUG)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MINCO_DEBUG)
endif()

set(_QPOASES_LINKED FALSE)

set(_QPOASES_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES)
if(EXISTS "${_QPOASES_VENDOR_DIR}/include/qpOASES.hpp" AND EXISTS "${_QPOASES_VENDOR_DIR}/src")
  file(GLOB _QPOASES_VENDOR_SRC "${_QPOASES_VENDOR_DIR}/src/*.cpp")
  add_library(qpOASES_vendor STATIC ${_QPOASES_VENDOR_SRC})
  set_target_properties(qpOASES_vendor PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(qpOASES_vendor PUBLIC "${_QPOASES_VENDOR_DIR}/include")
  target_compile_definitions(qpOASES_vendor PRIVATE __NO_COPYRIGHT__ LINUX)

  target_link_libraries(${PROJECT_NAME} qpOASES_vendor)
  set(_QPOASES_LINKED TRUE)
endif()

pluginlib_export_plugin_description_file(nav2_core minco_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_input_validation test/test_input_validation.cpp)
  if(TARGET test_input_validation)
    target_link_libraries(test_input_validation ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_mpc_solver test/test_mpc_solver.cpp)
  if(TARGET test_mpc_solver)
    target_link_libraries(test_mpc_solver ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_trajectory_session_gate test/test_trajectory_session_gate.cpp)
  if(TARGET test_trajectory_session_gate)
    target_link_libraries(test_trajectory_session_gate ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_reference_progress test/test_reference_progress.cpp)
  if(TARGET test_reference_progress)
    target_link_libraries(test_reference_progress ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_tilt_speed_limit test/test_tilt_speed_limit.cpp)
  if(TARGET test_tilt_speed_limit)
    target_include_directories(test_tilt_speed_limit PRIVATE include)
  endif()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
