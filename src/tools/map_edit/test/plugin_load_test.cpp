#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/tool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "map_edit/tool_manager.h"

TEST(MapEditPlugin, OffscreenClassesInstantiate) {
  int argc = 1;
  char app_name[] = "map_edit_plugin_test";
  char *argv[] = {app_name, nullptr};
  QApplication application(argc, argv);
  rclcpp::init(argc, argv);
  {
    pluginlib::ClassLoader<rviz_common::Tool> tools("rviz_common", "rviz_common::Tool");
    pluginlib::ClassLoader<rviz_common::Panel> panels("rviz_common", "rviz_common::Panel");
    auto eraser = tools.createSharedInstance("map_edit/MapEraserTool");
    auto region = tools.createSharedInstance("map_edit/RegionTool");
    auto panel = panels.createSharedInstance("map_edit/MapEditPanel");
    EXPECT_NE(eraser.get(), nullptr); EXPECT_NE(region.get(), nullptr); EXPECT_NE(panel.get(), nullptr);
    eraser.reset();
    region.reset();
    EXPECT_EQ(map_edit::ToolManager::getInstance().getMapEraserTool(), nullptr);
    EXPECT_EQ(map_edit::ToolManager::getInstance().getRegionTool(), nullptr);
    EXPECT_EQ(g_region_tool, nullptr);
    QApplication::processEvents();
  }
  rclcpp::shutdown();
}
