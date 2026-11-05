#include "rm27_backend/pointlio_adapter.hpp"
#include <algorithm>
#include <cstring>
#include <pcl/filters/voxel_grid.h>

namespace rm27_backend {
namespace {
/// 检查 ROS 时间范围并转换为整数纳秒，避免浮点时间比较误差。
std::int64_t ns(const builtin_interfaces::msg::Time &t) {
  if (t.sec < 0 || t.nanosec >= 1000000000U)
    throw std::invalid_argument("invalid timestamp");
  return std::int64_t(t.sec) * 1000000000LL + t.nanosec;
}
/// 查找并验证单个 FLOAT32 字段的字节偏移；可选字段缺失时返回 -1。
int field(const sensor_msgs::msg::PointCloud2 &c, const char *name,
          bool required = true) {
  for (const auto &f : c.fields)
    if (f.name == name) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count != 1 ||
          std::uint64_t(f.offset) + 4 > c.point_step)
        throw std::invalid_argument("invalid float field");
      return int(f.offset);
    }
  if (required)
    throw std::invalid_argument("missing XYZ field");
  return -1;
}
/// 安全读取可能未对齐的浮点字段，可选字段缺失时返回零。
float read(const std::uint8_t *p, int offset) {
  float v = 0;
  if (offset >= 0)
    std::memcpy(&v, p + offset, 4);
  return v;
}
} // namespace
/// 校验坐标名称、采样点数范围和体素尺寸。
PointlioAdapter::PointlioAdapter(AdapterConfig c) : config_(std::move(c)) {
  if (config_.world_frame.empty() || config_.sensor_frame.empty() ||
      config_.max_input_points < config_.max_cloud_points ||
      config_.min_cloud_points < 3 ||
      config_.max_cloud_points < config_.min_cloud_points ||
      !std::isfinite(config_.voxel_size) || config_.voxel_size < 0)
    throw std::invalid_argument("invalid adapter configuration");
}
/// 验收原子帧身份、位姿/点云同刻与同世界系约定，以及点云布局和资源上限。
/// 仅解析元数据；将 Point-LIO 的混合位置/旋转协方差转为局部右扰动，并标记输入质量。
Frame PointlioAdapter::decode(const Message &m) const {
  const auto &o = m.odometry;
  const auto &c = m.cloud;
  if (m.instance_id.empty() || m.instance_id.size() > 128 ||
      m.instance_id.find_first_not_of(
          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
          std::string::npos ||
      m.sequence == 0 || o.header.frame_id != config_.world_frame ||
      o.child_frame_id != config_.sensor_frame ||
      c.header.frame_id != config_.world_frame ||
      ns(o.header.stamp) != ns(c.header.stamp))
    throw std::invalid_argument(
        "invalid frame identity/coordinate/time contract");
  const std::uint64_t count = std::uint64_t(c.width) * c.height;
  if (c.is_bigendian || !count || count > config_.max_input_points ||
      !c.point_step || c.point_step > 128 ||
      std::uint64_t(c.row_step) < std::uint64_t(c.width) * c.point_step ||
      std::uint64_t(c.row_step) * c.height > c.data.size() ||
      c.data.size() > config_.max_input_points * 128ULL)
    throw std::invalid_argument("invalid cloud layout/resource bounds");
  field(c, "x");
  field(c, "y");
  field(c, "z");
  field(c, "intensity", false);
  const auto &p = o.pose.pose;
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y,
                       p.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.) > 1e-3)
    throw std::invalid_argument("invalid orientation");
  Frame f;
  f.identity = {m.instance_id, m.epoch};
  f.sequence = m.sequence;
  f.stamp_ns = ns(o.header.stamp);
  f.odom.linear() = q.normalized().toRotationMatrix();
  f.odom.translation() << p.position.x, p.position.y, p.position.z;
  if (!validPose(f.odom))
    throw std::invalid_argument("invalid pose");
  f.body_velocity << o.twist.twist.linear.x, o.twist.twist.linear.y,
      o.twist.twist.linear.z;
  if (!f.body_velocity.allFinite())
    throw std::invalid_argument("invalid velocity");
  Covariance raw;
  for (int r = 0; r < 6; ++r)
    for (int col = 0; col < 6; ++col)
      raw(r, col) = o.pose.covariance[r * 6 + col];
  // Point-LIO's documented mixed tangent: position world-additive, rotation
  // body-right. Convert to local-right [rho,phi]; do not reuse as edge noise.
  Covariance j = Covariance::Identity();
  j.topLeftCorner<3, 3>() = f.odom.linear().transpose();
  f.local_covariance = j * raw * j.transpose();
  f.covariance_valid = positiveCovariance(f.local_covariance);
  if (!f.covariance_valid)
    f.local_covariance.setZero();
  f.degraded = m.imu_saturated || m.rejected_imu_updates > 0 ||
               m.effective_points < config_.minimum_effective_points ||
               m.lidar_updates == 0;
  return f;
}
/// 复核消息与帧身份，用同帧位姿逆变换将去畸变世界云转换到 IMU 局部系。
/// 先等间隔限点，再去除无效点并体素过滤；有效点不足时拒绝创建关键帧。
Cloud::ConstPtr PointlioAdapter::cloud(const Message &m, const Frame &f) const {
  const auto &c = m.cloud;
  // decode() is repeated only at this public boundary to prevent unsafe callers
  // from bypassing layout validation; no point scan occurs there.
  const Frame verified = decode(m);
  if (verified.identity != f.identity || verified.sequence != f.sequence ||
      !verified.odom.matrix().isApprox(f.odom.matrix(), 1e-12))
    throw std::invalid_argument("cloud/frame mismatch");
  const int ox = field(c, "x"), oy = field(c, "y"), oz = field(c, "z"),
            oi = field(c, "intensity", false);
  const std::size_t count = std::size_t(c.width) * c.height;
  const std::size_t stride = std::max<std::size_t>(
      1, (count + config_.max_cloud_points - 1) / config_.max_cloud_points);
  Cloud::Ptr out(new Cloud);
  out->reserve(std::min(count, config_.max_cloud_points));
  const Pose inverse = f.odom.inverse();
  for (std::size_t i = 0; i < count; i += stride) {
    const auto *data = c.data.data() + (i / c.width) * c.row_step +
                       (i % c.width) * c.point_step;
    Eigen::Vector3d p(read(data, ox), read(data, oy), read(data, oz));
    if (!p.allFinite())
      continue;
    p = inverse * p;
    if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 1e5)
      continue;
    pcl::PointXYZI point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    point.intensity = read(data, oi);
    if (!std::isfinite(point.intensity))
      point.intensity = 0;
    out->push_back(point);
  }
  if (config_.voxel_size > 0 && !out->empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(out);
    voxel.setLeafSize(config_.voxel_size, config_.voxel_size,
                      config_.voxel_size);
    Cloud::Ptr filtered(new Cloud);
    voxel.filter(*filtered);
    out = filtered;
  }
  if (out->size() < config_.min_cloud_points)
    throw std::invalid_argument("insufficient valid keyframe points");
  out->is_dense = true;
  return out;
}
} // namespace rm27_backend
