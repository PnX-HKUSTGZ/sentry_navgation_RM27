#include <pcl/io/pcd_io.h>
#include <tbb/global_control.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>

#include "relocalization_manager/recovery_engine.hpp"
using namespace rm27_backend;
/// 离线几何验收入口：同一恢复库直接消费地图包与独立源云，输出机器可读结果。
int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 9) {
    std::cerr << "usage: recovery_probe BUNDLE SOURCE [x y z roll pitch yaw]\n";
    return 2;
  }
  try {
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, 2);
    const auto start = std::chrono::steady_clock::now();
    const auto manifest = YAML::LoadFile(std::string(argv[1]) + "/manifest.yaml");
    MatcherConfig m;
    m.gravity_aligned = true;
    m.min_inliers = 80;
    m.max_iterations = 30;
    m.max_rmse = .2;
    m.source_voxel = .2;
    ScanContextConfig sc;
    RecoveryConfig c;
    c.max_seconds = 20.;
    RecoveryEngine engine(
      argv[1], poseFromVector(manifest["map_from_pcd"].as<std::vector<double>>()), m, sc, c);
    const auto loaded = std::chrono::steady_clock::now();
    Cloud source;
    if (pcl::io::loadPCDFile(argv[2], source) != 0) return 3;
    std::optional<Pose> hint;
    if (argc == 9) {
      std::vector<double> p;
      for (int i = 3; i < 9; ++i) p.push_back(std::stod(argv[i]));
      hint = poseFromVector(p);
    }
    const auto result = engine.recover(source, Pose::Identity(), 1, hint, std::nullopt);
    const auto & p = result.match.pose;
    std::cout << "{\"success\":" << (result.found ? "true" : "false")
              << ",\"attempts\":" << result.attempts << ",\"reason\":\"" << result.reason
              << "\",\"rmse\":" << result.match.rmse << ",\"rank\":" << result.match.rank
              << ",\"xyz\":[" << p.translation().x() << "," << p.translation().y() << ","
              << p.translation().z() << "],\"quaternion_xyzw\":[";
    const Eigen::Quaterniond q(p.linear());
    std::cout << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "],\"load_ms\":"
              << std::chrono::duration<double, std::milli>(loaded - start).count()
              << ",\"recovery_ms\":"
              << std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - loaded)
                   .count()
              << "}" << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
