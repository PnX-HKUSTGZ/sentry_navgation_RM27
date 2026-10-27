// Export optimized SLAM Toolbox nodes and their original scan timestamps.
// This reads the native serialized graph, without GT or fitting another graph.
#include <karto_sdk/Mapper.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <sys/resource.h>

int main(int argc, char ** argv) {
  if (argc != 3) {
    std::cerr << "usage: slam_graph_export SERIALIZED_PREFIX OUTPUT.json\n";
    return 2;
  }
  try {
    // Boost's recursive graph archives need the same enlarged stack used by
    // SLAM Toolbox itself (stack_size_to_use), even in this standalone reader.
    rlimit stack;
    if (getrlimit(RLIMIT_STACK, &stack) != 0) throw std::runtime_error("cannot query stack limit");
    const rlim_t requested = 64 * 1024 * 1024;
    if (stack.rlim_cur < requested) {
      stack.rlim_cur = stack.rlim_max == RLIM_INFINITY ? requested : std::min(stack.rlim_max, requested);
      if (setrlimit(RLIMIT_STACK, &stack) != 0) throw std::runtime_error("cannot enlarge graph archive stack");
    }
    karto::Mapper mapper;
    mapper.LoadFromFile(std::string(argv[1]) + ".posegraph");
    nlohmann::json result = {{"nodes", nlohmann::json::array()},
                             {"edges", nlohmann::json::array()}, {"gt_used", false}};
    for (auto * scan : mapper.GetAllProcessedScans()) {
      const auto & odom = scan->GetOdometricPose();
      const auto & pose = scan->GetCorrectedPose();
      result["nodes"].push_back({{"id", scan->GetUniqueId()}, {"stamp", scan->GetTime()},
        {"odometry_xy_yaw", {odom.GetX(), odom.GetY(), odom.GetHeading()}},
        {"optimized_xy_yaw", {pose.GetX(), pose.GetY(), pose.GetHeading()}}});
    }
    for (auto * edge : mapper.GetGraph()->GetEdges()) {
      result["edges"].push_back({edge->GetSource()->GetObject()->GetUniqueId(),
                                edge->GetTarget()->GetObject()->GetUniqueId()});
    }
    std::ofstream output(argv[2]);
    if (!output || result["nodes"].empty()) throw std::runtime_error("empty graph or output unavailable");
    output << result.dump(2) << '\n';
    std::cout << "nodes=" << result["nodes"].size() << " edges=" << result["edges"].size() << '\n';
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 2;
  } catch (const karto::Exception & e) {
    std::cerr << e.GetErrorMessage() << '\n';
    return 2;
  }
  return 0;
}
