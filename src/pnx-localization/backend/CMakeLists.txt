cmake_minimum_required(VERSION 3.16)
project(rm27_backend LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rm27_localization_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)
# 优先使用系统安装的 GTSAM；文档中的用户目录安装脚本
# 用于无法取得管理员安装权限的环境。
find_package(GTSAM 4.2 QUIET)
if(NOT GTSAM_FOUND)
  find_package(GTSAM 4.2 REQUIRED PATHS
    "$ENV{HOME}/.local/opt/rm27-gtsam/usr/lib/cmake/GTSAM" NO_DEFAULT_PATH)
endif()
get_target_property(_gtsam_location gtsam IMPORTED_LOCATION_RELWITHDEBINFO)
if(_gtsam_location)
  get_filename_component(_gtsam_library_dir "${_gtsam_location}" DIRECTORY)
  set(CMAKE_BUILD_RPATH "${_gtsam_library_dir}")
  set(CMAKE_INSTALL_RPATH "${_gtsam_library_dir}")
endif()
# 允许加载用户目录中 GTSAM 的间接依赖库，例如 METIS。
if(UNIX AND NOT APPLE)
  add_link_options("LINKER:--disable-new-dtags")
endif()

add_library(rm27_backend_core src/pose_graph.cpp src/keyframe_manager.cpp src/cloud_store.cpp)
target_include_directories(rm27_backend_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS})
target_link_libraries(rm27_backend_core PUBLIC Eigen3::Eigen gtsam ${PCL_LIBRARIES} Threads::Threads TBB::tbb)
add_library(rm27_pointlio_adapter src/pointlio_adapter.cpp)
target_link_libraries(rm27_pointlio_adapter PUBLIC rm27_backend_core)
ament_target_dependencies(rm27_pointlio_adapter PUBLIC rm27_localization_interfaces sensor_msgs)
add_executable(rm27_backend_node apps/backend_node.cpp)
target_link_libraries(rm27_backend_node rm27_pointlio_adapter)
ament_target_dependencies(rm27_backend_node rclcpp nav_msgs diagnostic_msgs)

install(TARGETS rm27_backend_core rm27_pointlio_adapter EXPORT rm27_backend_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS rm27_backend_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include PATTERN ".gitkeep" EXCLUDE)
ament_export_targets(rm27_backend_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 GTSAM PCL Threads TBB rm27_localization_interfaces sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend rm27_pointlio_adapter)
endif()

install(FILES README.md VALIDATION.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
