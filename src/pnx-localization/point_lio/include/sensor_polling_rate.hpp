#pragma once

#include <rclcpp/rate.hpp>

namespace point_lio {
// Input polling is a local scheduling deadline, independent of both adjustable
// host realtime and the sensor/ROS timestamps used by the estimator.
using SensorPollingRate = rclcpp::WallRate;
}  // namespace point_lio
