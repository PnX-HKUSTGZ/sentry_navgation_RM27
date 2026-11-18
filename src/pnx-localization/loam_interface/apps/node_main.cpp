#include "loam_interface/loam_interface.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    // 两个执行线程分别服务反馈和点云，避免按逻辑核数创建大量线程。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    auto node = std::make_shared<loam_interface::LoamInterfaceNode>(rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
