#include <cstddef>
#include <memory>

#include <nav2_planner/planner_server.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

constexpr std::size_t kExecutorThreadCount = 8U;

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto planner_server =
    std::make_shared<nav2_planner::PlannerServer>(rclcpp::NodeOptions{});
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{}, kExecutorThreadCount);
  executor.add_node(planner_server->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
