#include "rm27_backend/map_localizer.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <unistd.h>

using namespace rm27_backend;
namespace {
/// 由平移和 ZYX 欧拉角构造位姿。
Pose pose(double x, double y, double z, double yaw, double pitch = 0,
          double roll = 0) {
  return poseFromVector({x, y, z, roll, pitch, yaw});
}
/// 以给定间距在矩形区域内生成平面网格点。
void grid(std::vector<Eigen::Vector3d> &out, const Eigen::Vector3d &origin,
          const Eigen::Vector3d &u, const Eigen::Vector3d &v, double lu,
          double lv, double step) {
  for (double a = 0; a <= lu + 1e-9; a += step)
    for (double b = 0; b <= lv + 1e-9; b += step)
      out.push_back(origin + a * u + b * v);
}
/// 12 m×8 m、高 2 m 的封闭房间，带三个不对称立柱，保证 6 自由度可观。
std::vector<Eigen::Vector3d> room() {
  std::vector<Eigen::Vector3d> p;
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX(), y = Eigen::Vector3d::UnitY(),
                        z = Eigen::Vector3d::UnitZ();
  grid(p, {-6, -4, 0}, x, y, 12, 8, .1);
  grid(p, {-6, -4, 0}, x, z, 12, 2, .1);
  grid(p, {-6, 4, 0}, x, z, 12, 2, .1);
  grid(p, {-6, -4, 0}, y, z, 8, 2, .1);
  grid(p, {6, -4, 0}, y, z, 8, 2, .1);
  for (const Eigen::Vector3d c : {Eigen::Vector3d(-3, 1.5, 0),
                                  Eigen::Vector3d(2, -2, 0),
                                  Eigen::Vector3d(4, 2.5, 0)})
    for (int side = 0; side < 4; ++side) {
      const Eigen::Vector3d d = side % 2 ? y : x;
      const Eigen::Vector3d o =
          c + (side == 1 ? Eigen::Vector3d(.6, 0, 0) : Eigen::Vector3d::Zero()) +
          (side == 2 ? Eigen::Vector3d(0, .6, 0) : Eigen::Vector3d::Zero());
      grid(p, o, d, z, .6, 1.5, .05);
    }
  return p;
}
/// 两侧墙加地面、两端开放的 40 m 长走廊，沿 map X 方向不可观。
std::vector<Eigen::Vector3d> corridor() {
  std::vector<Eigen::Vector3d> p;
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX(), y = Eigen::Vector3d::UnitY(),
                        z = Eigen::Vector3d::UnitZ();
  grid(p, {-20, -1.5, 0}, x, y, 40, 3, .1);
  grid(p, {-20, -1.5, 0}, x, z, 40, 2, .1);
  grid(p, {-20, 1.5, 0}, x, z, 40, 2, .1);
  return p;
}
/// 把 map 点按真实 map←IMU 位姿转到 IMU 局部系，只保留半径内的点并加 1 cm 噪声。
Cloud::Ptr observe(const std::vector<Eigen::Vector3d> &map, const Pose &truth,
                   double radius, std::mt19937 &rng) {
  std::normal_distribution<double> noise(0., .01);
  auto cloud = std::make_shared<Cloud>();
  const Pose inverse = truth.inverse();
  for (const auto &m : map) {
    const Eigen::Vector3d q = inverse * m;
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
/// 保存临时 PCD，供定位器后台加载。
std::string savePcd(const std::vector<Eigen::Vector3d> &points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto &p : points)
    cloud.emplace_back(float(p.x()), float(p.y()), float(p.z()));
  const auto path = std::filesystem::temp_directory_path() /
                    ("rm27_prior_map_" + std::to_string(getpid()) + ".pcd");
  pcl::io::savePCDFileBinary(path.string(), cloud);
  return path.string();
}
/// 关键帧触发间隔适配 10 Hz 仿真帧的里程计/图配置。
KeyframeManager manager() {
  KeyframeConfig k;
  k.distance = .5;
  k.max_interval = .5;
  GraphConfig g;
  g.lag_seconds = 0;
  g.max_states = 400;
  return KeyframeManager(k, g);
}
/// 绕房间中心半径 3 m 的圆轨迹上第 i 帧的真实 map←IMU 位姿，IMU 横滚 −30° 模拟倾斜雷达。
Pose truthAt(int i) {
  const double a = i * .1 / 3.;
  return pose(3 * std::cos(a), 3 * std::sin(a), .4, a + M_PI / 2, 0,
              -30. * M_PI / 180.);
}
/// 构造同一前端身份下的测试帧。
Frame frameAt(int i, const Pose &odom) {
  Frame f;
  f.identity = {"sim", 0};
  f.sequence = i + 1;
  f.stamp_ns = 1000000000LL + i * 100000000LL;
  f.odom = odom;
  return f;
}
/// 驱动定位器完成一条轨迹；jump_at≥0 时在该帧后给里程计加入 0.6 m 侧向跳变。
/// 返回最终图估计误差与纯里程计误差。
std::pair<double, double> run(MapLocalizer &localizer, int frames,
                              int jump_at = -1) {
  std::mt19937 rng(7);
  const auto map = room();
  const Pose alignment = pose(1., 2., 0., .35);  // 真实 map←world
  KeyframeManager km = manager();
  Pose last_odom = Pose::Identity();
  for (int i = 0; i < frames; ++i) {
    const Pose truth = truthAt(i);
    // 系统性漂移：1% 行程前向误差和 0.1°/m 偏航误差，单边约为里程计噪声模型的 1σ，
    // 持续同号累积；远超噪声模型的漂移会被 NIS 正确拒绝，不适合作为本测试输入。
    const double travelled = i * .1;
    Pose drift = pose(.01 * travelled, 0, 0, .1 * M_PI / 180. * travelled);
    if (jump_at >= 0 && i > jump_at)
      drift = pose(0, .6, 0, 0) * drift;
    const Pose odom = drift * alignment.inverse() * truth;
    last_odom = alignment * odom;
    Frame f = frameAt(i, odom);
    km.observe(f);
    // 与节点一致：每帧同步段号；未锚定时 observe 每帧都返回“新段”接纳结果。
    localizer.syncSegment(km.segment());
    if (!km.shouldCreate(f))
      continue;
    auto cloud = observe(map, truth, 8., rng);
    if (!km.latest()) {
      if (auto prior = localizer.takeAnchor(f)) {
        km.commit(f, prior);
      } else {
        // 种子带 0.2 m / 3° 误差，模拟导航定位链给出的近似对齐。
        localizer.requestAcquisition(f, cloud,
                                     pose(.2, -.1, 0, .05) * alignment * odom);
        localizer.waitIdle();
        localizer.apply(km);
      }
      continue;
    }
    const auto k = km.commit(f);
    localizer.requestTracking(k, k.estimate, cloud, 1);
    localizer.waitIdle();
    EXPECT_FALSE(localizer.apply(km).graph_failure);
  }
  const auto &latest = *km.latest();
  const Pose truth = truthAt(int((latest.frame.stamp_ns - 1000000000LL) / 100000000LL));
  const Pose raw = alignment * latest.frame.odom;
  return {(km.graph().estimate(latest.id).translation() - truth.translation()).norm(),
          (raw.translation() - truth.translation()).norm()};
}
/// 测试用定位器参数：每个关键帧都配准、立即重试锚定。
LocalizerConfig localizerConfig() {
  LocalizerConfig c;
  c.correction_period = 0;
  c.acquisition_period = 0;
  return c;
}
} // 匿名命名空间

TEST(Measurement, HessianRankConditionAndCommonModeFloor) {
  Matrix6 h = Matrix6::Zero();
  h.diagonal() << 100, 100, 0, 50, 50, 1e-5;
  auto analysis = analyzeHessian(h, 0);
  ASSERT_TRUE(analysis.valid);
  EXPECT_EQ(analysis.rank, 5); // 只剔除数值零空间
  analysis = analyzeHessian(h, 100);
  ASSERT_TRUE(analysis.valid);
  EXPECT_EQ(analysis.rank, 4); // 1e-5 方向弱于条件数门限
  const auto l = sqrtInformationFromHessian(analysis, 4.);
  ASSERT_EQ(l.rows(), 4);
  // LᵀL 在保留方向上恢复 s·H，在剔除方向上为零。
  const Matrix6 info = l.transpose() * l;
  EXPECT_NEAR(info(0, 0), 400, 1e-6);
  EXPECT_NEAR(info(3, 3), 200, 1e-6);
  EXPECT_NEAR(info(2, 2), 0, 1e-9);
  EXPECT_NEAR(info(5, 5), 0, 1e-9);
  Covariance floor = Covariance::Identity() * 1e-2;
  const auto bounded = addMeasurementCovariance(l, floor);
  ASSERT_TRUE(bounded);
  const Matrix6 reduced = bounded->transpose() * *bounded;
  // 400 的信息叠加 0.01 方差后为 1/(1/400+0.01)=80。
  EXPECT_NEAR(reduced(0, 0), 80, 1e-6);
  EXPECT_FALSE(covarianceFromSqrtInformation(*bounded)); // 秩不足没有协方差
  EXPECT_FALSE(analyzeHessian(Matrix6::Zero(), 10).valid);
}

TEST(Graph, MapFramePriorAndInnovationGate) {
  KeyframeManager km = manager();
  PosePrior prior;
  prior.pose = pose(5, -3, .4, 1.);
  prior.covariance = Covariance::Identity() * 1e-4;
  Frame f = frameAt(0, Pose::Identity());
  km.observe(f);
  const auto first = km.commit(f, prior);
  EXPECT_TRUE(first.estimate.isApprox(prior.pose, 1e-9)); // 图建立在 map 系
  Frame g = frameAt(10, pose(1, 0, 0, 0));
  km.observe(g);
  const auto second = km.commit(g);
  const Pose predicted = prior.pose * pose(1, 0, 0, 0);
  EXPECT_TRUE(second.estimate.isApprox(predicted, 1e-6));

  AbsolutePoseMeasurement m;
  m.key = second.id;
  m.segment = km.segment();
  m.sqrt_information = Eigen::MatrixXd::Identity(6, 6) * 100.;
  m.pose = predicted;
  auto innovation = km.graph().assessAbsolute(m);
  ASSERT_TRUE(innovation.valid);
  EXPECT_EQ(innovation.dof, 6);
  EXPECT_GT(innovation.probability, .5);
  m.pose = predicted * pose(0, 1., 0, 0); // 横向 1 m 冲突
  innovation = km.graph().assessAbsolute(m);
  ASSERT_TRUE(innovation.valid);
  EXPECT_LT(innovation.probability, 1e-6);
  m.sqrt_information = Eigen::MatrixXd::Identity(6, 6).topRows(3) * 100.;
  innovation = km.graph().assessAbsolute(m);
  EXPECT_EQ(innovation.dof, 3); // 部分秩观测按自身维数检验
  m.key = 999;
  EXPECT_FALSE(km.graph().assessAbsolute(m).valid);
}

TEST(PriorMap, RecoversTiltedSensorPoseWithFullRank) {
  MatcherConfig c;
  const auto map = PriorMap::fromPoints(room(), c);
  std::mt19937 rng(3);
  const Pose truth = pose(-1., .5, .4, .7, 0, -30. * M_PI / 180.);
  const auto cloud = observe(room(), truth, 8., rng);
  const auto result = map->match(*cloud, pose(.3, -.2, 0, .08) * truth, 1);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.rank, 6);
  EXPECT_LT((result.pose.translation() - truth.translation()).norm(), .02);
  EXPECT_LT(angle(truth.inverse() * result.pose), .005);
  EXPECT_LT(result.rmse, .03);
  EXPECT_TRUE(covarianceFromSqrtInformation(result.sqrt_information));
}

TEST(PriorMap, CorridorDropsBodyFrameAxialDirection) {
  MatcherConfig c;
  const auto map = PriorMap::fromPoints(corridor(), c);
  std::mt19937 rng(5);
  // 偏航 90°：map X 走廊轴在 IMU 局部右扰动中对应 Rᵀe_x。
  const Pose truth = pose(0, 0, .4, M_PI / 2);
  const auto cloud = observe(corridor(), truth, 8., rng);
  const auto result = map->match(*cloud, truth, 1);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(result.rank, 6);
  Vector6 axial = Vector6::Zero(), lateral = Vector6::Zero();
  axial.head<3>() = truth.linear().transpose() * Eigen::Vector3d::UnitX();
  lateral.head<3>() = truth.linear().transpose() * Eigen::Vector3d::UnitY();
  const double weak = (result.sqrt_information * axial).norm();
  const double strong = (result.sqrt_information * lateral).norm();
  EXPECT_GT(strong, 10.);
  EXPECT_LT(weak, .01 * strong);
}

TEST(MapLocalizer, AnchorsAndRemovesOdometryDrift) {
  const auto path = savePcd(room());
  MatcherConfig mc;
  MapLocalizer localizer(localizerConfig(), mc, path, Pose::Identity());
  localizer.waitIdle();
  ASSERT_TRUE(localizer.ready()) << localizer.reason();
  const auto [graph_error, odom_error] = run(localizer, 150);
  const auto stats = localizer.stats();
  EXPECT_EQ(stats.anchors, 1u);
  EXPECT_GT(stats.accepted, 20u);
  EXPECT_GT(odom_error, .15);  // 纯里程计已明显漂移
  EXPECT_LT(graph_error, .05); // 地图因子把图拉回真值
  std::filesystem::remove(path);
}

TEST(MapLocalizer, OdometryJumpTriggersSuspectThenConsistentRecovery) {
  const auto path = savePcd(room());
  MatcherConfig mc;
  MapLocalizer localizer(localizerConfig(), mc, path, Pose::Identity());
  localizer.waitIdle();
  ASSERT_TRUE(localizer.ready());
  const auto [graph_error, odom_error] = run(localizer, 170, 40);
  const auto stats = localizer.stats();
  EXPECT_GE(stats.nis_rejects + stats.correction_rejects, 7u);
  EXPECT_GE(stats.recoveries, 1u);
  EXPECT_EQ(localizer.state(), MapState::Tracking);
  EXPECT_GT(odom_error, .5);
  EXPECT_LT(graph_error, .05);
  std::filesystem::remove(path);
}

TEST(MapLocalizer, MissingMapFailsWithoutBlocking) {
  MapLocalizer localizer(localizerConfig(), MatcherConfig{}, "/nonexistent.pcd",
                         Pose::Identity());
  localizer.waitIdle();
  EXPECT_EQ(localizer.state(), MapState::Failed);
  EXPECT_FALSE(localizer.ready());
}
