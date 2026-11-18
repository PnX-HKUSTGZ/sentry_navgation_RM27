#include "rclcpp/rclcpp.hpp"
#include "sensor_scan_generation/sensor_scan_generation.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    // 同步点云处理与底盘反馈允许并发，各自回调组内部保持串行。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    auto node =
      std::make_shared<sensor_scan_generation::SensorScanGenerationNode>(rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
