#include "rm27_backend/cloud_store.hpp"
#include "rm27_backend/keyframe_manager.hpp"
#include "rm27_backend/pointlio_adapter.hpp"
#include <cstring>
#include <filesystem>
#include <gtest/gtest.h>
#include <unistd.h>

using namespace rm27_backend;
namespace {
/// 构造时间和序号递增的测试帧，可指定平移和偏航。
Frame frame(int i, double x = 0, double yaw = 0) {
  Frame f;
  f.identity = {"unit", 0};
  f.sequence = i + 1;
  f.stamp_ns = 1000000000LL + i * 100000000LL;
  f.odom.translation().x() = x;
  f.odom.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return f;
}
/// 构造带指定编号和 X 坐标的测试关键帧。
Keyframe key(int i, double x) {
  Keyframe k;
  k.id = i;
  k.frame = frame(i, x);
  return k;
}
/// 构造仅含 X 轴平移及已知对角协方差的测试里程计边。
OdometryMeasurement edge(int from, int to, double x) {
  OdometryMeasurement e;
  e.from = from;
  e.to = to;
  e.relative.translation().x() = x;
  e.covariance = Covariance::Identity() * .01;
  return e;
}
/// 构造同刻原子消息及规则网格点云，供坐标转换和消息验收测试使用。
PointlioAdapter::Message message() {
  PointlioAdapter::Message m;
  m.instance_id = "unit";
  m.sequence = 1;
  m.odometry.header.frame_id = "camera_init";
  m.odometry.child_frame_id = "body";
  m.odometry.header.stamp.sec = 1;
  m.odometry.pose.pose.position.x = 2;
  m.odometry.pose.pose.orientation.w = 1;
  for (int i = 0; i < 6; ++i)
    m.odometry.pose.covariance[i * 6 + i] = .01;
  m.lidar_updates = 1;
  m.effective_points = 100;
  auto &c = m.cloud;
  c.header = m.odometry.header;
  c.width = 100;
  c.height = 1;
  c.point_step = 16;
  c.row_step = 1600;
  c.data.resize(1600);
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = std::array<const char *, 4>{"x", "y", "z", "intensity"}[i];
    f.offset = i * 4;
    f.datatype = 7;
    f.count = 1;
    c.fields.push_back(f);
  }
  for (int i = 0; i < 100; ++i) {
    float p[4] = {float(2 + i % 10 * .2), float(i / 10 * .2), 1, 1};
    std::memcpy(c.data.data() + i * 16, p, 16);
  }
  return m;
}
} // namespace
/// 验证原子帧时间/坐标契约、局部点转换及畸形输入拒绝。
TEST(Adapter, AtomicTimeCoordinatesAndMalformedInputs) {
  PointlioAdapter a;
  auto m = message();
  auto f = a.decode(m);
  auto c = a.cloud(m, f);
  ASSERT_EQ(c->size(), 100U);
  EXPECT_NEAR(c->front().x, 0, 1e-6);
  EXPECT_TRUE(f.covariance_valid);
  EXPECT_FALSE(f.degraded);
  auto broken = m;
  broken.cloud.header.stamp.nanosec = 1;
  EXPECT_THROW(a.decode(broken), std::invalid_argument);
  broken = m;
  broken.cloud.data.resize(4);
  EXPECT_THROW(a.decode(broken), std::invalid_argument);
  broken = m;
  broken.cloud.fields[0].offset = 99;
  EXPECT_THROW(a.decode(broken), std::invalid_argument);
  broken = m;
  broken.odometry.pose.pose.orientation.w = 0;
  EXPECT_THROW(a.decode(broken), std::invalid_argument);
  broken = m;
  broken.cloud.header.frame_id = "body";
  EXPECT_THROW(a.decode(broken), std::invalid_argument);
  broken = m;
  broken.cloud.is_bigendian = true;
  EXPECT_THROW(a.decode(broken), std::invalid_argument);
  broken = m;
  broken.rejected_imu_updates = 1;
  EXPECT_TRUE(a.decode(broken).degraded);
}
/// 验证混合协方差的旋转转换、点数限制与无效点云拒绝。
TEST(Adapter, MixedCovarianceRotationAndCloudLimit) {
  AdapterConfig cfg;
  cfg.max_cloud_points = 40;
  cfg.min_cloud_points = 3;
  cfg.voxel_size = 0;
  PointlioAdapter a(cfg);
  auto m = message();
  m.odometry.pose.pose.orientation.w = std::sqrt(.5);
  m.odometry.pose.pose.orientation.z = std::sqrt(.5);
  m.odometry.pose.covariance[0] = .04;
  m.odometry.pose.covariance[7] = .09;
  auto f = a.decode(m);
  EXPECT_NEAR(f.local_covariance(0, 0), .09, 1e-10);
  EXPECT_NEAR(f.local_covariance(1, 1), .04, 1e-10);
  EXPECT_LE(a.cloud(m, f)->size(), 40U);
  m.odometry.pose.covariance[0] = -1;
  EXPECT_FALSE(a.decode(m).covariance_valid);
  float nan = std::numeric_limits<float>::quiet_NaN();
  for (int i = 0; i < 100; ++i)
    std::memcpy(m.cloud.data.data() + i * 16, &nan, 4);
  EXPECT_THROW(a.cloud(m, a.decode(m)), std::invalid_argument);
}
/// 验证关键帧运动/时间触发、重复拒绝及降级区间方差放大。
TEST(Keyframes, TriggersNoiseAndDuplicateRejection) {
  KeyframeManager m;
  auto f = frame(0);
  EXPECT_EQ(m.observe(f), Admission::NewSegment);
  m.commit(f);
  EXPECT_EQ(m.observe(f), Admission::Rejected);
  f = frame(1, .6);
  m.observe(f);
  EXPECT_FALSE(m.shouldCreate(f));
  f = frame(2, .6);
  f.degraded = true;
  m.observe(f);
  ASSERT_TRUE(m.shouldCreate(f));
  m.commit(f);
  ASSERT_TRUE(m.lastMeasurement());
  auto e = *m.lastMeasurement();
  EXPECT_NEAR(e.relative.translation().x(), .6, 1e-12);
  EXPECT_NEAR(e.covariance(0, 0), 10 * (.0005 * .2 + .0002 * .6), 1e-12);
  f = frame(4, .6, .3);
  m.observe(f);
  EXPECT_TRUE(m.shouldCreate(f));
  m.commit(f);
  f = frame(9, .6, .3);
  m.observe(f);
  EXPECT_TRUE(m.shouldCreate(f));
  m.commit(f);
  EXPECT_GT(m.sequenceGaps(), 0U);
}
/// 验证前端换代、进程更换和长间隔均不会建立跨段里程计边。
TEST(Keyframes, EpochInstanceAndGapNeverBridge) {
  KeyframeManager m;
  auto f = frame(0);
  m.observe(f);
  m.commit(f);
  f = frame(1);
  f.identity.epoch = 1;
  EXPECT_EQ(m.observe(f), Admission::NewSegment);
  m.commit(f);
  EXPECT_FALSE(m.lastMeasurement());
  EXPECT_EQ(m.observe(frame(2)), Admission::Rejected);
  f = frame(2);
  f.identity = {"new_process", 0};
  EXPECT_EQ(m.observe(f), Admission::NewSegment);
  m.commit(f);
  f = frame(3);
  EXPECT_EQ(m.observe(f), Admission::Rejected);
  f = frame(30);
  f.identity = {"new_process", 0};
  EXPECT_EQ(m.observe(f), Admission::NewSegment);
  m.commit(f);
  EXPECT_EQ(m.graph().size(), 1U);
  EXPECT_FALSE(m.lastMeasurement());
}
/// 验证估计器断段后仍保留输入水位，延迟旧帧不能重新进入图。
TEST(Keyframes, EstimatorResetPreservesInputWatermark) {
  KeyframeManager m;
  auto f = frame(5);
  m.observe(f);
  m.commit(f);
  m.breakSegment();
  EXPECT_EQ(m.observe(frame(4)), Admission::Rejected);
  f = frame(6);
  EXPECT_EQ(m.observe(f), Admission::NewSegment);
  m.commit(f);
  EXPECT_FALSE(m.lastMeasurement());
  EXPECT_EQ(m.keyframes().size(), 1U);
}
/// 验证协方差预测保留相对平移引起的位置/旋转耦合。
TEST(Keyframes, PredictionCovariancePreservesLeverArmCoupling) {
  KeyframeManager m;
  auto f = frame(0);
  m.observe(f);
  m.commit(f);
  const Covariance anchor = Covariance::Identity() * .01;
  EXPECT_TRUE(m.predictionCovariance(f, anchor).isApprox(anchor, 1e-12));
  f = frame(1, .1);
  m.observe(f);
  const auto p = m.predictionCovariance(f, anchor);
  EXPECT_TRUE(positiveCovariance(p));
  EXPECT_GT(p(1, 1), anchor(1, 1));
  EXPECT_NEAR(p(1, 5), .001, 1e-12);
}
/// 长链验证位姿复现、活动节点上限及因子空槽压缩。
TEST(Graph, ChainReproductionAndBoundedMarginalization) {
  GraphConfig c;
  c.max_states = 12;
  c.lag_seconds = .8;
  PoseGraph graph(c);
  graph.addFirst(key(0, 0));
  for (int i = 1; i <= 500; ++i) {
    graph.addOdometry(key(i, i * .1), edge(i - 1, i, .1));
    EXPECT_NEAR(graph.estimate(i).translation().x(), i * .1, 1e-7);
    EXPECT_LE(graph.size(), 12U);
  }
  EXPECT_GT(graph.marginalized(), 450U);
  EXPECT_LT(graph.factorSlots(), 50U);
  EXPECT_TRUE(positiveCovariance(graph.covariance(500)));
  EXPECT_FALSE(graph.contains(0));
}
/// 验证组合旋转和平移下相对位姿测量的方向正确。
TEST(Graph, RotationAndTranslationDirections) {
  PoseGraph graph;
  Keyframe first = key(0, 2);
  first.frame.odom.linear() =
      Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  graph.addFirst(first);
  auto second = key(1, 3);
  second.frame.odom.linear() =
      Eigen::AngleAxisd(-.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto e = edge(0, 1, 0);
  e.relative = first.frame.odom.inverse() * second.frame.odom;
  graph.addOdometry(second, e);
  EXPECT_TRUE(
      graph.estimate(1).matrix().isApprox(second.frame.odom.matrix(), 1e-9));
}
/// 对比完整图与裁剪/压缩图，检查位姿及边缘协方差的一致性。
TEST(Graph, MarginalizationAndCompactionPreserveCovariance) {
  GraphConfig bounded;
  bounded.max_states = 6;
  bounded.lag_seconds = 0;
  GraphConfig complete;
  complete.max_states = 0;
  complete.lag_seconds = 0;
  PoseGraph small(bounded), full(complete);
  small.addFirst(key(0, 0));
  full.addFirst(key(0, 0));
  for (int i = 1; i < 180; ++i) {
    auto k = key(i, i * .1);
    auto e = edge(i - 1, i, .1);
    small.addOdometry(k, e);
    full.addOdometry(k, e);
    if (i % 20 == 0) {
      EXPECT_TRUE(small.covariance(i).isApprox(full.covariance(i), 1e-7));
      EXPECT_TRUE(
          small.estimate(i).matrix().isApprox(full.estimate(i).matrix(), 1e-9));
    }
  }
}
/// 验证连续多次进程更换后，最早实例的延迟消息仍被拒绝。
TEST(Keyframes, OldInstancesRemainRejectedAfterMultipleRestarts) {
  KeyframeManager m;
  for (int i = 0; i < 3; ++i) {
    auto f = frame(i);
    f.identity.instance = "process" + std::to_string(i);
    m.observe(f);
    m.commit(f);
  }
  auto old = frame(10);
  old.identity.instance = "process0";
  EXPECT_EQ(m.observe(old), Admission::Rejected);
  EXPECT_EQ(m.graph().size(), 1U);
}
/// 验证地图/回环接口、部分秩信息、重复观测拒绝及过期节点处理。
TEST(Graph, ReservedMapLoopInterfacesAndStaleObservations) {
  GraphConfig c;
  c.max_states = 8;
  c.lag_seconds = 0;
  PoseGraph graph(c);
  graph.addFirst(key(0, 0));
  graph.addOdometry(key(1, 1), edge(0, 1, 1));
  graph.addOdometry(key(2, 2), edge(1, 2, 1));
  AbsolutePoseMeasurement map;
  map.measurement_id = "map_1";
  map.key = 2;
  map.pose.translation().x() = 1.8;
  map.sqrt_information = Eigen::MatrixXd::Identity(6, 6) * 10;
  EXPECT_TRUE(graph.addAbsolute(map));
  EXPECT_FALSE(graph.addAbsolute(map));
  EXPECT_LT(graph.estimate(2).translation().x(), 2.);
  LoopMeasurement loop;
  loop.measurement_id = "loop_1";
  loop.from = 0;
  loop.to = 2;
  loop.relative.translation().x() = 1.8;
  loop.sqrt_information = Eigen::MatrixXd::Identity(6, 6) * 10;
  EXPECT_TRUE(graph.addLoop(loop));
  EXPECT_FALSE(graph.addLoop(loop));
  map.measurement_id = "partial";
  map.sqrt_information = Eigen::MatrixXd::Zero(1, 6);
  map.sqrt_information(0, 0) = 10;
  EXPECT_TRUE(graph.addAbsolute(map));
  map.measurement_id = "bad";
  map.sqrt_information.setZero();
  EXPECT_THROW(graph.addAbsolute(map), std::invalid_argument);
  map.segment = 99;
  EXPECT_FALSE(graph.addAbsolute(map));
  for (int i = 3; i < 60; ++i)
    graph.addOdometry(key(i, i), edge(i - 1, i, 1));
  EXPECT_FALSE(graph.addLoop(loop));
  EXPECT_LE(graph.size(), 8U);
  EXPECT_TRUE(positiveCovariance(graph.covariance(59)));
}
/// 验证缓存逐出、归档加载、已有目录保护和磁盘配额错误统计。
TEST(Storage, CacheEvictionArchiveAndQuota) {
  auto path = std::filesystem::temp_directory_path() /
              ("rm27_storage_" + std::to_string(getpid()));
  std::filesystem::remove_all(path);
  StorageConfig c;
  c.cache_bytes = sizeof(pcl::PointXYZI) * 5;
  c.directory = path.string();
  c.queue_size = 10;
  {
    CloudStore s(c);
    Cloud::Ptr cloud(new Cloud);
    cloud->resize(5);
    for (auto &p : *cloud) {
      p.x = 1;
      p.y = 2;
      p.z = 3;
      p.intensity = 4;
    }
    for (int i = 0; i < 4; ++i)
      s.put(key(i, i), cloud, std::nullopt);
    s.flush();
    auto stats = s.stats();
    EXPECT_EQ(stats.saved, 4U);
    EXPECT_EQ(stats.errors, 0U);
    EXPECT_EQ(stats.cache_clouds, 1U);
    EXPECT_EQ(stats.evictions, 3U);
    ASSERT_TRUE(s.get(0));
    EXPECT_EQ(s.get(0)->size(), 5U);
    EXPECT_EQ(s.get(0)->front().z, 3);
    EXPECT_TRUE(std::filesystem::exists(path / "keyframes.csv"));
  }
  EXPECT_THROW(CloudStore duplicate(c), std::invalid_argument);
  std::filesystem::remove_all(path);
  c.disk_bytes = 1;
  {
    CloudStore s(c);
    Cloud::Ptr cloud(new Cloud);
    cloud->resize(5);
    s.put(key(0, 0), cloud, std::nullopt);
    s.flush();
    EXPECT_EQ(s.stats().errors, 1U);
  }
  std::filesystem::remove_all(path);
}
