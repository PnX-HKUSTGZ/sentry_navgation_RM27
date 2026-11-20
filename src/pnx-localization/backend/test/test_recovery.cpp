#include "rm27_backend/localization_controller.hpp"
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iomanip>
#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
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
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX(),
                        y = Eigen::Vector3d::UnitY(),
                        z = Eigen::Vector3d::UnitZ();
  grid(p, {-6, -4, 0}, x, y, 12, 8, .1);
  grid(p, {-6, -4, 0}, x, z, 12, 2, .1);
  grid(p, {-6, 4, 0}, x, z, 12, 2, .1);
  grid(p, {-6, -4, 0}, y, z, 8, 2, .1);
  grid(p, {6, -4, 0}, y, z, 8, 2, .1);
  for (const Eigen::Vector3d c :
       {Eigen::Vector3d(-3, 1.5, 0), Eigen::Vector3d(2, -2, 0),
        Eigen::Vector3d(4, 2.5, 0)})
    for (int side = 0; side < 4; ++side) {
      const Eigen::Vector3d d = side % 2 ? y : x;
      const Eigen::Vector3d o =
          c +
          (side == 1 ? Eigen::Vector3d(.6, 0, 0) : Eigen::Vector3d::Zero()) +
          (side == 2 ? Eigen::Vector3d(0, .6, 0) : Eigen::Vector3d::Zero());
      grid(p, o, d, z, .6, 1.5, .05);
    }
  return p;
}
/// 两侧墙加地面、两端开放的 40 m 长走廊，沿 map X 方向不可观。
std::vector<Eigen::Vector3d> corridor() {
  std::vector<Eigen::Vector3d> p;
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX(),
                        y = Eigen::Vector3d::UnitY(),
                        z = Eigen::Vector3d::UnitZ();
  grid(p, {-20, -1.5, 0}, x, y, 40, 3, .1);
  grid(p, {-20, -1.5, 0}, x, z, 40, 2, .1);
  grid(p, {-20, 1.5, 0}, x, z, 40, 2, .1);
  return p;
}
/// 把 map 点按真实 map←IMU 位姿转到 IMU 局部系，只保留半径内的点并加 1 cm
/// 噪声。
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
/// 测试文件摘要与生产地图包使用同一 SHA256 算法。
std::string digest(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(f)), {});
  unsigned char bytes[EVP_MAX_MD_SIZE];
  unsigned int count = 0;
  EVP_Digest(data.data(), data.size(), bytes, &count, EVP_sha256(), nullptr);
  std::ostringstream out;
  for (unsigned int i = 0; i < count; ++i)
    out << std::hex << std::setw(2) << std::setfill('0') << int(bytes[i]);
  return out.str();
}
/// 在隔离临时目录生成非对称房间的三视点地图包，退出自动清理。
class Bundle {
public:
  /// 构造地图与局部参考云；查询使用不同位置和独立噪声。
  Bundle() {
    static int number = 0;
    path = std::filesystem::temp_directory_path() /
           ("rm27_recovery_test_" + std::to_string(getpid()) + "_" +
            std::to_string(number++));
    std::filesystem::create_directories(path);
    const auto points = room();
    std::mt19937 rng(31);
    auto global = observe(points, Pose::Identity(), 100, rng);
    pcl::io::savePCDFileBinary((path / "map.pcd").string(), *global);
    YAML::Node m;
    m["format"] = "rm27_reference_map_v1";
    m["frame"] = "map";
    m["version"] = "test-v1";
    m["coverage"]["p95_m"] = .3;
    m["map"] = "map.pcd";
    m["map_sha256"] = digest((path / "map.pcd").string());
    m["map_from_pcd"] = std::vector<double>(6, 0.);
    for (int i = 0; i < 3; ++i) {
      std::vector<double> p = {double(i * 2 - 2), .3, .4, 0., 0., 0.};
      const std::string file = "view" + std::to_string(i) + ".pcd";
      auto cloud = observe(points, poseFromVector(p), 10, rng);
      pcl::io::savePCDFileBinary((path / file).string(), *cloud);
      YAML::Node view;
      view["pose"] = p;
      view["cloud"] = file;
      view["sha256"] = digest((path / file).string());
      m["views"].push_back(view);
    }
    std::ofstream(path / "manifest.yaml") << m;
  }
  /// 删除本测试创建的临时文件。
  ~Bundle() { std::filesystem::remove_all(path); }
  std::filesystem::path path;
};
/// 测试配准参数保持严格厘米级残差，点数下限适配合成输入。
MatcherConfig matcher() {
  MatcherConfig m;
  m.min_inliers = 80;
  m.max_iterations = 30;
  return m;
}
/// 构造同代连续扫描元数据。
Frame frame(int index) {
  Frame f;
  f.identity = {"test", 0};
  f.sequence = index + 1;
  f.stamp_ns = 1000000000LL + index * 100000000LL;
  return f;
}
} // namespace
/// 未知航向恢复必须由独立点云配准得到完整地图位姿。
TEST(Recovery, UnknownYawAndWrongHint) {
  Bundle b;
  RecoveryConfig c;
  c.max_seconds = 20;
  c.candidates = 3;
  RecoveryEngine e(b.path.string(), Pose::Identity(), matcher(),
                   ScanContextConfig{}, c);
  std::mt19937 rng(82);
  const Pose truth = pose(.25, .13, .4, .8);
  const auto source = observe(room(), truth, 10, rng);
  auto result =
      e.recover(*source, Pose::Identity(), 1, std::nullopt, std::nullopt);
  ASSERT_TRUE(result.found) << result.reason;
  EXPECT_LT((result.match.pose.translation() - truth.translation()).norm(),
            .05);
  EXPECT_LT(angle(truth.inverse() * result.match.pose), .015);
  auto hinted = e.recover(*source, Pose::Identity(), 1, pose(100, 100, 10, 0),
                          std::nullopt);
  ASSERT_TRUE(hinted.found) << hinted.reason;
  EXPECT_LT((hinted.match.pose.translation() - truth.translation()).norm(),
            .05);
}
/// 退化平面和空云不得宣布完整六维恢复。
TEST(Recovery, EmptyAndDegenerateSourcesRejected) {
  Bundle b;
  RecoveryConfig c;
  c.max_seconds = 20;
  RecoveryEngine e(b.path.string(), Pose::Identity(), matcher(),
                   ScanContextConfig{}, c);
  Cloud empty;
  EXPECT_FALSE(
      e.recover(empty, Pose::Identity(), 1, std::nullopt, std::nullopt).found);
  Cloud plane;
  for (float x = -4; x < 4; x += .1)
    for (float y = -3; y < 3; y += .1) {
      pcl::PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = -.4;
      plane.push_back(p);
    }
  EXPECT_FALSE(
      e.recover(plane, Pose::Identity(), 1, pose(0, 0, .4, 0), std::nullopt)
          .found);
}
/// 地图变换错误、产物篡改和文件缺失均在加载阶段失败。
TEST(Recovery, BundleIdentityAndIntegrity) {
  Bundle b;
  EXPECT_THROW(
      RecoveryEngine(b.path.string(), pose(1, 0, 0, 0), matcher(), {}, {}),
      std::runtime_error);
  {
    std::ofstream f(b.path / "view0.pcd", std::ios::app);
    f << "corrupt";
  }
  EXPECT_THROW(
      RecoveryEngine(b.path.string(), Pose::Identity(), matcher(), {}, {}),
      std::runtime_error);
}
/// 在后台完成前切换工作代次，旧恢复结果不得创建图锚。
TEST(RecoveryController, StaleResultAndVerifiedAnchor) {
  Bundle b;
  ControllerConfig c;
  c.recovery.max_seconds = 20;
  c.result_max_age = 2;
  LocalizationController controller(c, matcher(), {}, b.path.string(),
                                    Pose::Identity());
  controller.waitIdle();
  ASSERT_TRUE(controller.ready());
  KeyframeManager km;
  controller.syncSegment(km.segment());
  std::mt19937 rng(33);
  const auto cloud = observe(room(), pose(0, .3, .4, 0), 10, rng);
  controller.manual(pose(0, .3, .4, 0));
  for (int i = 0; i < 2; ++i) {
    km.observe(frame(i));
    controller.observe(frame(i), cloud);
  }
  ASSERT_TRUE(controller.requestRecovery(frame(1)));
  controller.manual(std::nullopt);
  controller.waitIdle();
  controller.apply(km);
  EXPECT_FALSE(controller.takeAnchor(frame(1)));
  EXPECT_FALSE(controller.state().trusted());
  controller.manual(pose(0, .3, .4, 0));
  for (int i = 2; i < 4; ++i) {
    km.observe(frame(i));
    controller.observe(frame(i), cloud);
  }
  ASSERT_TRUE(controller.requestRecovery(frame(3)));
  controller.waitIdle();
  controller.apply(km);
  const auto anchor = controller.takeAnchor(frame(3));
  ASSERT_TRUE(anchor);
  EXPECT_FALSE(controller.state().trusted());
  km.commit(frame(3), anchor);
  controller.confirmAnchor();
  EXPECT_TRUE(controller.state().trusted());
  controller.inputTimedOut();
  EXPECT_FALSE(controller.state().trusted());
  const auto generation = controller.state().generation();
  controller.inputTimedOut();
  EXPECT_EQ(generation, controller.state().generation());
}
/// 地图缺失时绝不退回带 map 标记的纯里程计正常输出。
TEST(RecoveryController, MissingMapRemainsUntrusted) {
  LocalizationController controller(
      {}, matcher(), {}, "/nonexistent/rm27/bundle", Pose::Identity());
  controller.waitIdle();
  EXPECT_TRUE(controller.failed());
  EXPECT_FALSE(controller.ready());
  EXPECT_FALSE(controller.state().trusted());
}

/// 两个几何相同的房间必须拒绝唯一定位，不能用描述子排序或最先收敛冒充确定性。
TEST(Recovery, SymmetricRoomsAreRejected) {
  Bundle bundle;
  const auto root = bundle.path;
  auto manifest = YAML::LoadFile((root / "manifest.yaml").string());
  Cloud map;
  pcl::io::loadPCDFile((root / "map.pcd").string(), map);
  const auto first = map;
  for (auto p : first) {
    p.x += 30;
    map.push_back(p);
  }
  pcl::io::savePCDFileBinary((root / "map.pcd").string(), map);
  manifest["map_sha256"] = digest((root / "map.pcd").string());
  for (int i = 0; i < 3; ++i) {
    auto view = YAML::Clone(manifest["views"][i]);
    auto p = view["pose"].as<std::vector<double>>();
    p[0] += 30;
    view["pose"] = p;
    manifest["views"].push_back(view);
  }
  {
    std::ofstream file(root / "manifest.yaml");
    file << manifest;
  }
  RecoveryConfig c;
  c.candidates = 6;
  c.max_seconds = 20;
  RecoveryEngine engine(root.string(), Pose::Identity(), matcher(), {}, c);
  std::mt19937 rng(55);
  const auto source = observe(room(), pose(.25, .1, .4, .4), 10, rng);
  const auto result =
      engine.recover(*source, Pose::Identity(), 1, std::nullopt, std::nullopt);
  EXPECT_FALSE(result.found);
  EXPECT_NE(result.reason.find("ambiguous"), std::string::npos)
      << result.reason;
}

/// 稀疏参考地图在加载时拒绝，避免未知地点检索漏掉真实窄通道。
TEST(Recovery, UndersampledMapRejected) {
  Bundle b;
  auto manifest = YAML::LoadFile((b.path / "manifest.yaml").string());
  manifest["coverage"]["p95_m"] = 2.;
  {
    std::ofstream f(b.path / "manifest.yaml");
    f << manifest;
  }
  EXPECT_THROW(
      RecoveryEngine(b.path.string(), Pose::Identity(), matcher(), {}, {}),
      std::runtime_error);
}

/// 倾斜 IMU 的重力初值经过描述子整平与反变换后仍保持正确安装角。
TEST(Recovery, TiltedSensorUsesOdometryGravity) {
  Bundle b;
  auto m = matcher();
  m.gravity_aligned = true;
  RecoveryConfig c;
  c.max_seconds = 20;
  RecoveryEngine engine(b.path.string(), Pose::Identity(), m, {}, c);
  const Pose truth = pose(.25, .1, .4, .8, -.1, -.52);
  const Pose odom = pose(0, 0, 0, 0, -.1, -.52);
  std::mt19937 rng(77);
  const auto cloud = observe(room(), truth, 10, rng);
  const auto result =
      engine.recover(*cloud, odom, 1, std::nullopt, std::nullopt);
  ASSERT_TRUE(result.found) << result.reason;
  EXPECT_LT((result.match.pose.translation() - truth.translation()).norm(),
            .05);
  EXPECT_LT(angle(truth.inverse() * result.match.pose), .02);
}

/// 预处理源复用必须与原接口得到相同结果，取消不能产出可用测量。
TEST(Recovery, PreparedSourceAndIterationCancellation) {
  Bundle b;
  auto m = matcher();
  m.point_to_plane = true;
  auto target =
      PriorMap::load((b.path / "map.pcd").string(), Pose::Identity(), m);
  std::mt19937 rng(311);
  const Pose truth = pose(.25, .1, .4, .3);
  const auto cloud = observe(room(), truth, 10, rng);
  const auto prepared = PriorMap::prepareSource(*cloud, m);
  const auto a = target->match(*cloud, truth, 1);
  const auto c = target->match(*prepared, truth, 1);
  ASSERT_TRUE(a.valid) << a.reason;
  ASSERT_TRUE(c.valid) << c.reason;
  EXPECT_LT((a.pose.translation() - c.pose.translation()).norm(), 1e-8);
  int checks = 0;
  const auto cancelled =
      target->match(*prepared, truth, 1, [&] { return ++checks > 4; });
  EXPECT_FALSE(cancelled.valid);
  EXPECT_NE(cancelled.reason.find("cancelled"), std::string::npos);
  EXPECT_GT(checks, 4);
}

/// 有限提示可以选择拒绝全图回退；错误初值不能偷偷变成另一地点的定位成功。
TEST(Recovery, HintBoundsAndBudget) {
  Bundle b;
  auto m = matcher();
  m.point_to_plane = true;
  RecoveryConfig config;
  config.hint_global_fallback = false;
  RecoveryEngine engine(b.path.string(), Pose::Identity(), m, {}, config);
  std::mt19937 rng(312);
  const Pose truth = pose(.25, .1, .4, .3);
  const auto cloud = observe(room(), truth, 10, rng);
  const auto right =
      engine.recover(*cloud, Pose::Identity(), 1, truth, std::nullopt);
  ASSERT_TRUE(right.found) << right.reason;
  const auto wrong = engine.recover(*cloud, Pose::Identity(), 1,
                                    pose(100, 100, 10, 0), std::nullopt);
  EXPECT_FALSE(wrong.found);
  EXPECT_EQ(wrong.attempts, config.hint_yaw_samples);
  const auto cancelled =
      engine.recover(*cloud, Pose::Identity(), 1, std::nullopt, std::nullopt,
                     [] { return true; });
  EXPECT_FALSE(cancelled.found);
  EXPECT_TRUE(cancelled.deferred);
  EXPECT_EQ(cancelled.attempts, 0);
  config.max_seconds = 1e-9;
  RecoveryEngine limited(b.path.string(), Pose::Identity(), m, {}, config);
  const auto expired =
      limited.recover(*cloud, Pose::Identity(), 1, truth, std::nullopt);
  EXPECT_FALSE(expired.found);
  EXPECT_TRUE(expired.deferred);
}

/// 手动请求复用同代扫描；等待 TF 时不启动无提示全局搜索，取消后必须等待新请求。
TEST(RecoveryController, ReuseHistoryWaitForHintAndCancel) {
  Bundle b;
  ControllerConfig config;
  auto m = matcher();
  m.point_to_plane = true;
  LocalizationController controller(config, m, {}, b.path.string(),
                                    Pose::Identity());
  controller.waitIdle();
  KeyframeManager manager;
  controller.syncSegment(manager.segment());
  std::mt19937 rng(313);
  const Pose truth = pose(0, .3, .4, 0);
  const auto cloud = observe(room(), truth, 10, rng);
  for (int i = 0; i < 2; ++i) {
    manager.observe(frame(i));
    controller.observe(frame(i), cloud);
  }
  controller.manual(std::nullopt, std::nullopt, true);
  EXPECT_FALSE(controller.requestRecovery(frame(1)));
  controller.updateHint(truth, frame(1).odom);
  ASSERT_TRUE(controller.requestRecovery(frame(1)));
  // 同一状态代次内更新提示也必须撤销在途结果，不只依赖 generation 判断。
  const auto generation = controller.state().generation();
  controller.updateHint(truth, frame(1).odom);
  EXPECT_EQ(generation, controller.state().generation());
  controller.waitIdle();
  controller.apply(manager);
  EXPECT_FALSE(controller.takeAnchor(frame(1)));
  ASSERT_TRUE(controller.requestRecovery(frame(1)));
  controller.waitIdle();
  controller.apply(manager);
  ASSERT_TRUE(controller.takeAnchor(frame(1)));
  controller.cancelRecovery("test deadline");
  EXPECT_FALSE(controller.takeAnchor(frame(1)));
  EXPECT_FALSE(controller.requestRecovery(frame(1)));
  EXPECT_FALSE(controller.state().trusted());
  controller.manual(truth);
  EXPECT_TRUE(controller.requestRecovery(frame(1)));
  controller.waitIdle();
}

/// 可信锚与同刻前端位姿组合必须保留旋转、杆臂参考点和运动方向。
TEST(MotionRecovery, PredictsRelativeMotionAndGrowsEnvelope) {
  MotionRecovery motion;
  Frame first = frame(0);
  first.odom = pose(4, 2, .3, .5);
  motion.observe(first);
  const Pose anchor = pose(-3, 1, .4, 1.);
  motion.anchor(anchor, first);
  auto current = frame(1);
  current.odom = first.odom * pose(.2, 0, 0, .1);
  motion.observe(current);
  const auto predicted = motion.predict(current);
  ASSERT_TRUE(predicted);
  const Pose expected = anchor * first.odom.inverse() * current.odom;
  EXPECT_LT((predicted->pose.matrix() - expected.matrix()).norm(), 1e-9);
  EXPECT_GT(predicted->radius, 1.5);
  EXPECT_GT(predicted->angle, .261799);
  EXPECT_FALSE(predicted->contains(pose(30, 0, 0, 0)));
  const double radius = predicted->radius;
  motion.observe(current);
  EXPECT_DOUBLE_EQ(motion.predict(current)->radius, radius);
}

/// 前端质量、时间、身份、速度和年龄失效后，后续健康帧不能复活旧可信锚。
TEST(MotionRecovery, InvalidityDoesNotResurrectOldAnchor) {
  for (int scenario = 0; scenario < 6; ++scenario) {
    MotionRecoveryConfig config;
    config.max_age = .15;
    MotionRecovery motion(config);
    auto initial = frame(0);
    motion.observe(initial);
    motion.anchor(Pose::Identity(), initial);
    auto next = frame(1);
    if (scenario == 0)
      next.identity.epoch = 1;
    if (scenario == 1)
      next.degraded = true;
    if (scenario == 2)
      next.stamp_ns = initial.stamp_ns - 1;
    if (scenario == 3)
      next.odom = pose(10, 0, 0, 0);
    if (scenario == 4)
      next.stamp_ns += 2000000000LL;
    if (scenario == 5) {
      next = frame(2);
    }
    motion.observe(next);
    EXPECT_FALSE(motion.predict(next)) << scenario;
    auto healthy = next;
    healthy.sequence++;
    healthy.stamp_ns += 100000000LL;
    healthy.degraded = false;
    motion.observe(healthy);
    EXPECT_FALSE(motion.predict(healthy)) << scenario;
  }
  MotionRecovery disabled(MotionRecoveryConfig{false});
  auto f = frame(0);
  disabled.observe(f);
  disabled.anchor(Pose::Identity(), f);
  EXPECT_FALSE(disabled.predict(f));
}

/// 使用累计路程而非首尾净位移扩大包络，往返运动不能低估漂移。
TEST(MotionRecovery, RoundTripAccumulatesDistanceAndManualInvalidation) {
  MotionRecovery motion;
  auto a = frame(0);
  motion.observe(a);
  motion.anchor(Pose::Identity(), a);
  auto b = frame(1);
  b.odom = pose(.5, 0, 0, 0);
  motion.observe(b);
  auto c = frame(2);
  motion.observe(c);
  auto region = motion.predict(c);
  ASSERT_TRUE(region);
  EXPECT_GT(region->radius, 1.5 + .15 * .2 + .079);
  motion.invalidate("manual relocation");
  EXPECT_FALSE(motion.predict(c));
}

/// 两个对称房间在无连续性时必须拒绝；有效运动范围仅允许真实房间内几何结果。
TEST(Recovery, MotionEnvelopeRejectsOppositeSymmetricRoom) {
  Bundle bundle;
  auto manifest = YAML::LoadFile((bundle.path / "manifest.yaml").string());
  Cloud map;
  pcl::io::loadPCDFile((bundle.path / "map.pcd").string(), map);
  const auto first = map;
  for (auto p : first) {
    p.x += 30;
    map.push_back(p);
  }
  pcl::io::savePCDFileBinary((bundle.path / "map.pcd").string(), map);
  manifest["map_sha256"] = digest((bundle.path / "map.pcd").string());
  for (int i = 0; i < 3; ++i) {
    auto view = YAML::Clone(manifest["views"][i]);
    auto v = view["pose"].as<std::vector<double>>();
    v[0] += 30;
    view["pose"] = v;
    manifest["views"].push_back(view);
  }
  {
    std::ofstream f(bundle.path / "manifest.yaml");
    f << manifest;
  }
  auto m = matcher();
  m.point_to_plane = true;
  RecoveryConfig c;
  c.max_seconds = 20;
  c.candidates = 6;
  RecoveryEngine engine(bundle.path.string(), Pose::Identity(), m, {}, c);
  const Pose truth = pose(.25, .1, .4, .4);
  std::mt19937 rng(611);
  const auto source = observe(room(), truth, 10, rng);
  const auto global_start = std::chrono::steady_clock::now();
  const auto global =
      engine.recover(*source, Pose::Identity(), 1, std::nullopt, std::nullopt);
  const auto global_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - global_start)
                             .count();
  EXPECT_FALSE(global.found);
  EXPECT_NE(global.reason.find("ambiguous"), std::string::npos);
  RecoveryRegion region;
  region.pose = pose(.5, .2, .4, .5);
  region.radius = 1.;
  region.angle = .3;
  const auto local_start = std::chrono::steady_clock::now();
  const auto local = engine.recover(*source, Pose::Identity(), 1, std::nullopt,
                                    std::nullopt, {}, {}, region);
  const auto local_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - local_start)
                            .count();
  RecordProperty("unbounded_ms", std::to_string(global_ms));
  RecordProperty("bounded_ms", std::to_string(local_ms));
  RecordProperty("unbounded_attempts", global.attempts);
  RecordProperty("bounded_attempts", local.attempts);
  ASSERT_TRUE(local.found) << local.reason;
  EXPECT_TRUE(region.contains(local.match.pose));
  EXPECT_LT((local.match.pose.translation() - truth.translation()).norm(), .05);
  EXPECT_LE(local.attempts, 2);
  // 无约束版本可能在两个地点就提前发现歧义；计算次数不是成功率门限。
  EXPECT_GE(global.attempts, 2);
}

/// LOST 阶段也必须以运动增量更新搜索中心；不允许退化为静态最后位置搜索。
TEST(RecoveryController, LostRecoveryUsesMovingContinuityAnchor) {
  Bundle bundle;
  auto m = matcher();
  m.point_to_plane = true;
  ControllerConfig config;
  LocalizationController controller(config, m, {}, bundle.path.string(),
                                    Pose::Identity());
  controller.waitIdle();
  KeyframeManager manager;
  controller.syncSegment(manager.segment());
  const Pose truth = pose(0, .3, .4, .2);
  std::mt19937 rng(612);
  for (int i = 0; i < 2; ++i) {
    auto f = frame(i);
    manager.observe(f);
    controller.observe(f, observe(room(), truth, 10, rng));
  }
  controller.manual(truth);
  ASSERT_TRUE(controller.requestRecovery(frame(1)));
  controller.waitIdle();
  controller.apply(manager);
  auto anchor = controller.takeAnchor(frame(1));
  ASSERT_TRUE(anchor);
  manager.commit(frame(1), anchor);
  controller.confirmAnchor();
  controller.rememberPose(truth, frame(1).stamp_ns);
  controller.manual(std::nullopt);
  EXPECT_EQ(controller.state().status(), LocalizationStatus::Lost);
  Frame latest;
  for (int i = 2; i <= 31; ++i) {
    latest = frame(i);
    latest.odom = pose((i - 1) * .1, 0, 0, 0);
    manager.observe(latest);
    controller.observe(latest, observe(room(), truth * latest.odom, 10, rng));
  }
  ASSERT_TRUE(controller.requestRecovery(latest));
  controller.waitIdle();
  controller.apply(manager);
  const auto recovered = controller.takeAnchor(latest);
  ASSERT_TRUE(recovered) << controller.reason();
  EXPECT_LT(
      (recovered->pose.translation() - (truth * latest.odom).translation())
          .norm(),
      .05);
  EXPECT_TRUE(controller.stats().motion_recovery_active);
  EXPECT_LT(controller.stats().motion_recovery_radius, 3.);
  controller.inputTimedOut();
  auto f = latest;
  f.sequence++;
  f.stamp_ns += 100000000LL;
  manager.observe(f);
  controller.observe(f, observe(room(), truth * f.odom, 10, rng));
  // 单周期限流通过后，断流后的恢复不再附带旧运动约束。
  f.sequence += 10;
  f.stamp_ns += 1000000000LL;
  manager.observe(f);
  controller.observe(f, observe(room(), truth * f.odom, 10, rng));
  f.sequence++;
  f.stamp_ns += 100000000LL;
  manager.observe(f);
  controller.observe(f, observe(room(), truth * f.odom, 10, rng));
  EXPECT_FALSE(controller.requestRecovery(f));
  EXPECT_FALSE(controller.stats().motion_recovery_active);
  EXPECT_NE(controller.reason().find("unbounded automatic recovery blocked"),
            std::string::npos);
  controller.manual(std::nullopt);
  ASSERT_TRUE(controller.requestRecovery(f));
  controller.waitIdle();
}

/// 真正执行点到面匹配和图提交，避免仅测试一个与实现相同的计数器。
class RecoveryPolicyTest : public ::testing::Test {
protected:
  void SetUp() override {
    auto m = matcher();
    m.point_to_plane = true;
    m.max_rmse = .05;
    ControllerConfig config;
    config.rmse_failures_before_suspect = rmseLimit();
    config.tracking.correction_period = .01;
    config.tracking.acquisition_period = .01;
    controller = std::make_unique<LocalizationController>(
        config, m, ScanContextConfig{}, bundle.path.string(), Pose::Identity());
    controller->waitIdle();
    controller->syncSegment(manager.segment());
    controller->manual(truth);
    for (int i = 0; i < 2; ++i) {
      manager.observe(frame(i));
      controller->observe(frame(i), observe(room(), truth, 10, rng));
    }
    ASSERT_TRUE(controller->requestRecovery(frame(1)));
    controller->waitIdle();
    controller->apply(manager);
    auto prior = controller->takeAnchor(frame(1));
    ASSERT_TRUE(prior) << controller->reason();
    manager.commit(frame(1), prior);
    controller->confirmAnchor();
    controller->rememberPose(truth, frame(1).stamp_ns);
  }

  virtual int rmseLimit() const { return 3; }

  void track(bool noisy) {
    latest = frame(next++);
    ASSERT_NE(manager.observe(latest), Admission::Rejected);
    auto cloud = observe(room(), truth, 10, rng);
    if (noisy) {
      std::normal_distribution<double> error(0., .3);
      for (auto &p : *cloud) {
        p.x += error(rng);
        p.y += error(rng);
        p.z += error(rng);
      }
    }
    controller->observe(latest, cloud);
    const auto key = manager.commit(latest);
    ASSERT_TRUE(controller->requestTracking(key, key.estimate, cloud, 1));
    controller->waitIdle();
    accepted = controller->apply(manager).accepted;
  }

  Bundle bundle;
  KeyframeManager manager{[] {
    KeyframeConfig config;
    config.min_interval = 0.;
    config.max_interval = .05;
    return config;
  }()};
  std::unique_ptr<LocalizationController> controller;
  Pose truth = pose(0, .3, .4, 0);
  std::mt19937 rng{912};
  Frame latest;
  int next = 2;
  std::size_t accepted = 0;
};

TEST_F(RecoveryPolicyTest,
       ThirdRmseRejectionTriggersSuspectAndBoundedRecovery) {
  for (int i = 1; i <= 3; ++i) {
    track(true);
    EXPECT_EQ(accepted, 0U);
    EXPECT_EQ(controller->stats().consecutive_rmse_failures, i);
    EXPECT_EQ(controller->state().trusted(), i < 3);
    // 同一结果不能因再次 apply 而重复计数。
    controller->apply(manager);
    EXPECT_EQ(controller->stats().consecutive_rmse_failures, i);
  }
  EXPECT_EQ(controller->state().status(), LocalizationStatus::Suspect);
  // 健康前端仍有运动包络，关闭无范围恢复不应封死附近恢复。
  // 使用后续独立正常扫描刷新恢复窗口，不把故意加噪的跟踪云当作恢复源。
  for (int i = 0; i < 5; ++i) {
    latest = frame(next++);
    manager.observe(latest);
    controller->observe(latest, observe(room(), truth, 10, rng));
  }
  ASSERT_TRUE(controller->requestRecovery(latest));
  EXPECT_TRUE(controller->stats().motion_recovery_active);
  controller->waitIdle();
  controller->apply(manager);
  auto prior = controller->takeAnchor(latest);
  ASSERT_TRUE(prior) << controller->reason();
  manager.commit(latest, prior);
  controller->confirmAnchor();
  EXPECT_TRUE(controller->state().trusted());
  EXPECT_EQ(controller->stats().consecutive_rmse_failures, 0U);
}

TEST_F(RecoveryPolicyTest, AcceptedMapFactorAndNewSegmentClearRmseFailures) {
  track(true);
  track(true);
  ASSERT_EQ(controller->stats().consecutive_rmse_failures, 2U);
  track(false);
  ASSERT_EQ(accepted, 1U);
  EXPECT_EQ(controller->stats().consecutive_rmse_failures, 0U);
  track(true);
  track(true);
  EXPECT_TRUE(controller->state().trusted());
  manager.breakSegment();
  controller->syncSegment(manager.segment());
  EXPECT_EQ(controller->stats().consecutive_rmse_failures, 0U);
}

TEST_F(RecoveryPolicyTest, ManualRequestClearsFailureEvidence) {
  track(true);
  controller->manual(truth);
  EXPECT_EQ(controller->stats().consecutive_rmse_failures, 0U);
  EXPECT_FALSE(controller->state().trusted());
}

TEST_F(RecoveryPolicyTest, TimeoutClearsFailureEvidence) {
  track(true);
  ASSERT_EQ(controller->stats().consecutive_rmse_failures, 1U);
  controller->inputTimedOut();
  EXPECT_EQ(controller->stats().consecutive_rmse_failures, 0U);
  EXPECT_FALSE(controller->state().trusted());
}

class ImmediateRmsePolicyTest : public RecoveryPolicyTest {
protected:
  int rmseLimit() const override { return 1; }
};

TEST_F(ImmediateRmsePolicyTest, ThresholdOneRetainsImmediateRejection) {
  track(true);
  EXPECT_EQ(controller->stats().consecutive_rmse_failures, 1U);
  EXPECT_EQ(controller->state().status(), LocalizationStatus::Suspect);
}

TEST(RecoveryController, UnboundedAutomaticRecoveryRequiresOptIn) {
  Bundle bundle;
  std::mt19937 rng(913);
  const auto cloud = observe(room(), pose(0, .3, .4, 0), 10, rng);
  for (bool allow : {false, true}) {
    ControllerConfig config;
    config.allow_unbounded_auto_recovery = allow;
    LocalizationController controller(config, matcher(), {},
                                      bundle.path.string(), Pose::Identity());
    controller.waitIdle();
    KeyframeManager manager;
    controller.syncSegment(manager.segment());
    for (int i = 0; i < 2; ++i) {
      manager.observe(frame(i));
      controller.observe(frame(i), cloud);
    }
    EXPECT_EQ(controller.requestRecovery(frame(1)), allow);
    if (!allow) {
      EXPECT_EQ(controller.stats().requests, 0U);
      EXPECT_NE(
          controller.reason().find("unbounded automatic recovery blocked"),
          std::string::npos);
    }
    // 显式无提示服务保留全图入口，不能被自动恢复策略误禁用。
    controller.manual(std::nullopt);
    controller.waitIdle();
    ASSERT_TRUE(controller.requestRecovery(frame(1)));
    controller.waitIdle();
  }
}

TEST(RecoveryController, RejectsInvalidRmseFailureThreshold) {
  for (int limit : {0, -1}) {
    ControllerConfig config;
    config.rmse_failures_before_suspect = limit;
    EXPECT_THROW(LocalizationController(config, matcher(), {}, "unused",
                                        Pose::Identity()),
                 std::invalid_argument);
  }
}
