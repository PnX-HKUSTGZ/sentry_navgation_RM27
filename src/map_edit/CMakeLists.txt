cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
project(map_edit)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Quick Qml QuickWidgets)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# MOC 文件（用于 QObject 继承类）
qt5_wrap_cpp(MOC_FILES
  include/map_edit/map_edit_panel.h
  include/map_edit/map_eraser_tool.h
  include/map_edit/region_tool.h

)

# 源文件
set(SRC_FILES
  src/map_edit_panel.cpp
  src/tool_manager.cpp
  src/map_eraser_tool.cpp
  src/region_tool.cpp
  src/map_document.cpp
  src/pcd_loader.cpp
)

# 头文件包含路径
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

# 构建共享库（rviz 插件要求）
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${MOC_FILES}
)

# Reject unresolved plugin/MOC symbols while building the shared library itself.
if(UNIX AND NOT APPLE AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--no-undefined")
endif()

# 链接 Qt 和 ROS 所需库
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Quick
  Qt5::Qml
  Qt5::QuickWidgets
  yaml-cpp
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  nav2_map_server
  tf2_geometry_msgs
  rviz_common
  ament_index_cpp
  nav_msgs
  tf2_ros
  rviz_rendering
  cv_bridge
  tf2
  visualization_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
)

# 安装共享库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)


# 安装插件描述文件（pluginlib）
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(map_document_test test/map_document_test.cpp src/map_document.cpp)
  target_include_directories(map_document_test PRIVATE include)
  target_compile_definitions(map_document_test PRIVATE MAP_EDIT_REPOSITORY_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/../pb2025_nav_bringup/map/simulation/RMUC2026.yaml")
  target_link_libraries(map_document_test Qt5::Widgets yaml-cpp)
  ament_target_dependencies(map_document_test nav2_map_server nav_msgs tf2 tf2_geometry_msgs)
  ament_add_gtest(plugin_load_test test/plugin_load_test.cpp)
  target_link_libraries(plugin_load_test Qt5::Widgets ${PROJECT_NAME})
  ament_target_dependencies(plugin_load_test pluginlib rviz_common rclcpp)
  set_tests_properties(plugin_load_test PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endif()
install(FILES UPSTREAM.txt DESTINATION share/${PROJECT_NAME})
ament_package()
