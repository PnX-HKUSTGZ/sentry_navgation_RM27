#pragma once
#include "rm27_backend/types.hpp"
#include <rm27_localization_interfaces/msg/frontend_frame.hpp>

namespace rm27_backend {
/// Point-LIO 输入坐标约定、点数上限、关键帧采样和质量判定参数。
struct AdapterConfig {
  std::string world_frame = "camera_init", sensor_frame = "body";
  std::size_t max_input_points = 300000, max_cloud_points = 12000,
              min_cloud_points = 30;
  double voxel_size = .15;
  std::size_t minimum_effective_points = 20;
};
/// 将 Point-LIO 原子扫描消息适配为后端元数据和 IMU 局部点云。
/// 元数据验收与点云处理分开，普通帧无需执行完整点云转换。
class PointlioAdapter {
public:
  using Message = rm27_localization_interfaces::msg::FrontendFrame;
  /// 校验坐标名称、点数范围和体素参数。
  explicit PointlioAdapter(AdapterConfig config = {});
  /// 验收消息身份、同刻约定及点云布局，转换位姿协方差并标记质量。
  Frame decode(const Message &message) const;
  /// 验证帧与点云配对，将世界云转换到参考时刻 IMU 系，再限点和体素采样。
  Cloud::ConstPtr cloud(const Message &message, const Frame &frame) const;

private:
  AdapterConfig config_;
};
} // 命名空间 rm27_backend
