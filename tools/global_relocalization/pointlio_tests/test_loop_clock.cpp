#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Rate system_rate(500);
  rclcpp::WallRate scheduling_rate(500);
  if (system_rate.get_type() != RCL_SYSTEM_TIME ||
    scheduling_rate.get_type() != RCL_STEADY_TIME)
  {
    throw std::runtime_error("mapping loop must use the steady scheduling clock");
  }
  scheduling_rate.sleep();
  rclcpp::shutdown();
  std::cout << "mapping loop scheduling uses RCL_STEADY_TIME\n";
}
