#include "rm27_backend/loop_closer.hpp"
#include <gtest/gtest.h>
#include <random>

using namespace rm27_backend;
namespace {
/// 由平移和 ZYX 欧拉角构造位姿。
Pose pose(double x, double y, double z, double yaw, double pitch = 0, double roll = 0) {
  return poseFromVector({x, y, z, roll, pitch, yaw});
}
/// 在矩形区域内按步长生成平面网格点。
void grid(std::vector<Eigen::Vector3d> &out, const Eigen::Vector3d &o,
          const Eigen::Vector3d &u, const Eigen::Vector3d &v, double lu, double lv,
          double step) {
  for (double a = 0; a <= lu + 1e-9; a += step)
    for (double b = 0; b <= lv + 1e-9; b += step)
      out.push_back(o + a * u + b * v);
}
/// 14 m×10 m 封闭房间，内有四个不对称立柱，供回环几何验证。
std::vector<Eigen::Vector3d> room() {
  std::vector<Eigen::Vector3d> p;
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX(), y = Eigen::Vector3d::UnitY(),
                        z = Eigen::Vector3d::UnitZ();
  grid(p, {-7, -5, 0}, x, y, 14, 10, .1);
  grid(p, {-7, -5, 0}, x, z, 14, 2, .1);
  grid(p, {-7, 5, 0}, x, z, 14, 2, .1);
  grid(p, {-7, -5, 0}, y, z, 10, 2, .1);
  grid(p, {7, -5, 0}, y, z, 10, 2, .1);
  for (const Eigen::Vector3d c : {Eigen::Vector3d(-4.5, 2, 0), Eigen::Vector3d(1, -3.5, 0),
                                  Eigen::Vector3d(4.5, 3, 0), Eigen::Vector3d(0, 1.5, 0)})
    for (int side = 0; side < 4; ++side) {
      const Eigen::Vector3d d = side % 2 ? y : x;
      const Eigen::Vector3d o = c + (side == 1 ? Eigen::Vector3d(.6, 0, 0) : Eigen::Vector3d::Zero()) +
                                (side == 2 ? Eigen::Vector3d(0, .6, 0) : Eigen::Vector3d::Zero());
      grid(p, o, d, z, .6, 1.5, .05);
    }
  return p;
}
/// 按真实 world←IMU 位姿把世界点转到 IMU 局部系，保留半径内的点并加 1 cm 噪声。
Cloud::Ptr observe(const std::vector<Eigen::Vector3d> &world, const Pose &truth,
                   double radius, std::mt19937 &rng) {
  std::normal_distribution<double> noise(0., .01);
  auto cloud = std::make_shared<Cloud>();
  const Pose inverse = truth.inverse();
  for (const auto &w : world) {
    const Eigen::Vector3d q = inverse * w;
    if (q.norm() > radius)
      continue;
    pcl::PointXYZI point;
    point.x = q.x() + noise(rng);
    point.y = q.y() + noise(rng);
    point.z = q.z() + noise(rng);
    point.intensity = 0;
    cloud->push_back(point);
  }
  return cloud;
}
/// 绕 (0.5, 0) 半径 3.5 m 的圆轨迹第 i 帧真值；IMU 横滚 −30° 模拟倾斜安装雷达。
Pose truthAt(int i) {
  const double a = i * .1 / 3.5;
  return pose(.5 + 3.5 * std::cos(a), 3.5 * std::sin(a), .4, a + M_PI / 2, 0,
              -30. * M_PI / 180.);
}
/// 构造同一前端身份的测试帧。
Frame frameAt(int i, const Pose &odom) {
  Frame f;
  f.identity = {"loop", 0};
  f.sequence = i + 1;
  f.stamp_ns = 1000000000LL + i * 100000000LL;
  f.odom = odom;
  return f;
}
/// 回环模式的关键帧/图配置：全图、0.5 m 关键帧。
KeyframeManager loopManager() {
  KeyframeConfig k;
  k.distance = .5;
  k.max_interval = .5;
  GraphConfig g;
  g.lag_seconds = 0;
  g.max_states = 3000;
  g.loop_cauchy_size = .5;
  return KeyframeManager(k, g);
}
} // 匿名命名空间

TEST(ScanContext, RecoversYawAndSeparatesPlaces) {
  ScanContextConfig c;
  c.exclude_recent = 1;
  c.augment_count = 1;
  ScanContextDatabase db(c);
  std::mt19937 rng(1);
  const auto world = room();
  const Pose target = pose(-2, 0, .4, 0);
  db.add(*observe(world, target, 30, rng));               // 0：目标位置
  db.add(*observe(world, pose(5, 3.5, .4, 0), 30, rng));  // 1：另一位置
  // 同一位置、机体偏航 +40°：source→target 偏航应为 +40°。
  const double yaw = 40. * M_PI / 180.;
  db.add(*observe(world, target * pose(0, 0, 0, yaw), 30, rng));
  const auto candidates = db.detectLatest({0, 1});
  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(candidates.front().index, 0);
  EXPECT_NEAR(candidates.front().yaw, yaw, 4.1 * M_PI / 180.); // 一个扇区 4°
  if (candidates.size() > 1)
    EXPECT_LT(candidates[0].distance, candidates[1].distance);
}

TEST(Graph, LoopInnovationUsesJointMarginal) {
  KeyframeManager km = loopManager();
  for (int i = 0; i <= 60; i += 5) {
    Frame f = frameAt(i, pose(i * .1, 0, 0, 0));
    km.observe(f);
    km.commit(f);
  }
  LoopMeasurement m;
  m.from = 0;
  m.to = km.latest()->id;
  m.segment = km.segment();
  m.sqrt_information = Eigen::MatrixXd::Identity(6, 6) * 50.;
  m.relative = pose(6, 0, 0, 0);
  auto innovation = km.graph().assessLoop(m);
  ASSERT_TRUE(innovation.valid);
  EXPECT_GT(innovation.probability, .5);
  m.relative = pose(6, 2., 0, 0); // 2 m 横向闭合差远超里程计不确定度
  innovation = km.graph().assessLoop(m);
  ASSERT_TRUE(innovation.valid);
  EXPECT_LT(innovation.probability, 1e-6);
  m.to = 999;
  EXPECT_FALSE(km.graph().assessLoop(m).valid);
}

/// 两圈带漂移的合成场景：检出、验证并入图回环，最终误差应明显小于原始里程计。
void runRevisit(const LoopConfig &lc) {
  ScanContextConfig sc;
  sc.lidar_height = .4;
  MatcherConfig mc;
  mc.map_voxel = mc.source_voxel = lc.cloud_voxel;
  mc.min_inliers = 80;
  mc.max_rmse = .25;
  mc.translation_std_floor = .005;
  mc.rotation_std_floor = .02 * M_PI / 180.;
  LoopCloser loop(lc, sc, mc);
  KeyframeManager km = loopManager();
  std::mt19937 rng(9);
  const auto world = room();
  const int frames = 2 * 220 + 40; // 两圈多一点
  for (int i = 0; i < frames; ++i) {
    const Pose truth = truthAt(i);
    // 世界系左乘漂移：1% 行程前向误差和 0.02°/m 偏航误差。整圈闭合差须在里程计噪声模型
    // 允许范围内，否则联合 NIS 会按设计拒绝（与 z-localization 相同的噪声默认值）。
    const double travelled = i * .1;
    const Pose odom = pose(.01 * travelled, 0, 0, .02 * M_PI / 180. * travelled) * truth;
    Frame f = frameAt(i, odom);
    km.observe(f);
    loop.syncSegment(km.segment());
    EXPECT_FALSE(loop.apply(km).graph_failure);
    if (!km.shouldCreate(f))
      continue;
    const auto k = km.commit(f);
    loop.addKeyframe(k, *observe(world, truth, 8, rng));
    loop.request(km);
    loop.waitIdle();
  }
  loop.apply(km);
  const auto stats = loop.stats();
  EXPECT_GE(stats.accepted, 3u);
  EXPECT_EQ(stats.nis_rejects, 0u);
  // 图规范为首帧原始位姿，等于真值首帧，可直接与真值比较。
  const auto &latest = *km.latest();
  const int index = int((latest.frame.stamp_ns - 1000000000LL) / 100000000LL);
  const Pose truth = truthAt(index);
  const double odom_error = (latest.frame.odom.translation() - truth.translation()).norm();
  const double loop_error =
      (km.graph().estimate(latest.id).translation() - truth.translation()).norm();
  EXPECT_GT(odom_error, .4);
  EXPECT_LT(loop_error, .5 * odom_error);
  EXPECT_LT(loop_error, .15);
}

TEST(LoopCloser, DetectsVerifiesAndReducesDriftOnRevisit) { runRevisit(LoopConfig{}); }

TEST(LoopCloser, SamePassTargetAndQuerySpacingStillCloseLoops) {
  LoopConfig lc;
  lc.target_same_pass = true;
  lc.query_distance = 1.;
  runRevisit(lc);
}

TEST(LoopCloser, StationaryQueriesFollowDistanceAndInterval) {
  const auto world = room();
  /// 静止 20 s（输入 2 Hz、每秒定时建一个关键帧），返回检索次数。
  const auto queries = [&](double interval) {
    LoopConfig lc;
    lc.query_distance = 1.;
    lc.query_interval = interval;
    ScanContextConfig sc;
    sc.exclude_recent = 1;
    LoopCloser loop(lc, sc, MatcherConfig{});
    KeyframeConfig k;
    k.max_interval = 1.;
    KeyframeManager km(k, GraphConfig{});
    std::mt19937 rng(3);
    for (int i = 0; i < 40; ++i) {
      Frame f = frameAt(5 * i, truthAt(0));
      km.observe(f);
      if (!km.shouldCreate(f))
        continue;
      loop.addKeyframe(km.commit(f), *observe(world, truthAt(0), 8, rng));
      loop.request(km);
      loop.waitIdle();
      loop.apply(km);
    }
    return loop.stats().queries;
  };
  EXPECT_EQ(queries(0.), 1u);   // 只按行程：静止时首次之后不再检索
  EXPECT_EQ(queries(5.), 4u);   // 第 1、6、11、16 个关键帧各检索一次
}

TEST(LoopCloser, SegmentChangeClearsDatabase) {
  LoopCloser loop(LoopConfig{}, ScanContextConfig{}, MatcherConfig{});
  KeyframeManager km = loopManager();
  std::mt19937 rng(2);
  const auto world = room();
  EXPECT_FALSE(loop.syncSegment(0));
  for (int i = 0; i < 20; i += 5) {
    Frame f = frameAt(i, truthAt(i));
    km.observe(f);
    loop.addKeyframe(km.commit(f), *observe(world, truthAt(i), 8, rng));
  }
  EXPECT_EQ(loop.stats().keyframes, 4u);
  EXPECT_FALSE(loop.syncSegment(0));
  EXPECT_TRUE(loop.syncSegment(1));
  EXPECT_EQ(loop.stats().keyframes, 0u);
}
