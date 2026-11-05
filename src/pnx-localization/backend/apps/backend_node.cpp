#include "rm27_backend/cloud_store.hpp"
#include "rm27_backend/keyframe_manager.hpp"
#include "rm27_backend/pointlio_adapter.hpp"
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <tbb/global_control.h>
#include <unistd.h>

namespace rm27_backend {
/// 装配后端 ROS 输入、关键帧/图管理、存储与旁路输出的节点。
/// 由单线程执行器串行处理图状态；磁盘归档由 CloudStore 独立线程完成，不广播导航 TF。
class BackendNode : public rclcpp::Node {
public:
  /// 声明并校验参数，设置线程预算，依次创建核心模块、订阅和诊断定时器。
  /// 启用存盘时使用唯一运行目录，避免不同记录互相覆盖。
  BackendNode() : Node("rm27_backend") {
    const bool enabled = declare_parameter("enabled", true);
    const auto topic = declare_parameter(
        "input.frame_topic", std::string("/point_lio/backend_frame"));
    AdapterConfig a;
    a.world_frame = declare_parameter("input.world_frame", a.world_frame);
    a.sensor_frame = declare_parameter("input.sensor_frame", a.sensor_frame);
    output_frame_ =
        declare_parameter("output.frame", std::string("rm27_graph"));
    sensor_frame_ = a.sensor_frame;
    a.max_input_points = positive("input.max_points", a.max_input_points);
    a.max_cloud_points = positive("cloud.max_points", a.max_cloud_points);
    a.min_cloud_points = positive("cloud.min_points", a.min_cloud_points);
    a.voxel_size = declare_parameter("cloud.voxel_size", a.voxel_size);
    a.minimum_effective_points = positive("quality.minimum_effective_points",
                                          a.minimum_effective_points);
    KeyframeConfig k;
    k.distance = declare_parameter("keyframe.distance", k.distance);
    k.angle_rad = declare_parameter("keyframe.angle_deg", 10.) * M_PI / 180.;
    k.min_interval = declare_parameter("keyframe.min_interval", k.min_interval);
    k.max_interval = declare_parameter("keyframe.max_interval", k.max_interval);
    k.max_gap = declare_parameter("input.max_gap", k.max_gap);
    k.max_linear_speed =
        declare_parameter("input.max_linear_speed", k.max_linear_speed);
    k.max_angular_speed =
        declare_parameter("input.max_angular_speed", k.max_angular_speed);
    k.translation_variance_per_second = declare_parameter(
        "odom_noise.translation_per_second", k.translation_variance_per_second);
    k.translation_variance_per_meter = declare_parameter(
        "odom_noise.translation_per_meter", k.translation_variance_per_meter);
    k.rotation_variance_per_second = declare_parameter(
        "odom_noise.rotation_per_second", k.rotation_variance_per_second);
    k.rotation_variance_per_radian = declare_parameter(
        "odom_noise.rotation_per_radian", k.rotation_variance_per_radian);
    k.degraded_noise_multiplier = declare_parameter(
        "quality.degraded_noise_multiplier", k.degraded_noise_multiplier);
    GraphConfig g;
    g.lag_seconds = declare_parameter("graph.lag_seconds", g.lag_seconds);
    g.max_states = positive("graph.max_states", g.max_states);
    g.prior_translation_std = declare_parameter("graph.prior_translation_std",
                                                g.prior_translation_std);
    g.prior_rotation_std =
        declare_parameter("graph.prior_rotation_std", g.prior_rotation_std);
    g.relinearize_threshold = declare_parameter("graph.relinearize_threshold",
                                                g.relinearize_threshold);
    StorageConfig s;
    s.cache_bytes = positive("storage.cache_mb", 32) * 1024 * 1024;
    s.queue_size = positive("storage.queue_size", 4);
    s.disk_bytes = positive("storage.disk_mb", 1024) * 1024ULL * 1024;
    s.directory = declare_parameter("storage.directory", std::string(""));
    const auto threads = positive("graph.num_threads", 2);
    const auto depth = positive("input.queue_depth", 3);
    if (depth > 32 || g.max_states > 5000 || a.max_input_points > 1000000 ||
        a.max_cloud_points > 200000 || s.cache_bytes > 2048ULL * 1024 * 1024 ||
        s.queue_size > 128 || threads > 64)
      throw std::invalid_argument(
          "resource parameters exceed supported bounds");
    threads_ = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, threads);
    adapter_ = std::make_unique<PointlioAdapter>(a);
    manager_ = std::make_unique<KeyframeManager>(k, g);
    if (!enabled) {
      RCLCPP_INFO(get_logger(), "Backend disabled by configuration");
      return;
    }
    if (!s.directory.empty()) {
      const auto run =
          std::chrono::system_clock::now().time_since_epoch().count();
      s.directory +=
          "/run_" + std::to_string(run) + "_" + std::to_string(getpid());
    }
    store_ = std::make_unique<CloudStore>(s);
    estimate_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/keyframes", 1);
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "~/diagnostics", 10);
    subscription_ = create_subscription<PointlioAdapter::Message>(
        topic, rclcpp::SensorDataQoS().keep_last(depth),
        // 将原子帧交给统一处理入口，保持接纳与图更新串行。
        [this](PointlioAdapter::Message::ConstSharedPtr m) { receive(*m); });
    // 定时回调只发布统计，不推进图状态。
    timer_ =
        create_wall_timer(std::chrono::seconds(1), [this] { diagnostics(); });
    RCLCPP_INFO(
        get_logger(),
        "Odometry-only iSAM2 backend ready; output=%s, no TF; archive=%s",
        output_frame_.c_str(),
        s.directory.empty() ? "disabled" : s.directory.c_str());
  }

private:
  /// 声明正整数参数并拒绝零或负数。
  std::size_t positive(const char *name, std::size_t value) {
    auto n = declare_parameter<std::int64_t>(name, value);
    if (n <= 0)
      throw std::invalid_argument(std::string(name) + " must be positive");
    return std::size_t(n);
  }
  /// 将 Eigen 位姿写入 ROS 的位置和四元数字段。
  static void poseMessage(const Pose &p, geometry_msgs::msg::Pose &m) {
    Eigen::Quaterniond q(p.linear());
    m.position.x = p.translation().x();
    m.position.y = p.translation().y();
    m.position.z = p.translation().z();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
  }
  /// 处理一条原子扫描：验收身份与连续性，按需准备点云并提交关键帧和图更新。
  /// 利用最近图锚投影当前帧并传播协方差；图提交异常会断段，其他异常记录为输入拒绝。
  void receive(const PointlioAdapter::Message &m) {
    const auto start = std::chrono::steady_clock::now();
    ++received_;
    try {
      Frame f = adapter_->decode(m);
      const auto admission = manager_->observe(f);
      if (admission == Admission::Rejected) {
        ++rejected_;
        last_reason_ = manager_->reason();
        return;
      }
      if (admission == Admission::NewSegment) {
        ++segments_;
        last_reason_ = manager_->reason();
      }
      last_stamp_ = m.odometry.header.stamp;
      ++accepted_;
      if (f.degraded)
        ++degraded_;
      if (manager_->shouldCreate(f)) {
        // Geometry must be ready before committing the graph/keyframe pair.
        auto cloud = adapter_->cloud(m, f);
        const auto graph_start = std::chrono::steady_clock::now();
        Keyframe k;
        try {
          k = manager_->commit(f);
        } catch (...) {
          manager_->breakSegment();
          ++graph_failures_;
          throw;
        }
        graph_ms_ = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - graph_start)
                        .count();
        graph_ms_max_ = std::max(graph_ms_max_, graph_ms_);
        store_->put(k, cloud, manager_->lastMeasurement());
        last_cloud_points_ = cloud->size();
        nav_msgs::msg::Path path;
        path.header.stamp = last_stamp_;
        path.header.frame_id =
            output_frame_ + "_s" + std::to_string(manager_->segment());
        for (const auto &[id, p] : manager_->graph().estimates()) {
          geometry_msgs::msg::PoseStamped item;
          item.header = path.header;
          item.header.stamp = rclcpp::Time(manager_->graph().stamp(id));
          poseMessage(p, item.pose);
          path.poses.push_back(item);
        }
        path_pub_->publish(path);
        last_covariance_ = manager_->graph().covariance(k.id);
      }
      if (manager_->latest()) {
        const auto &k = *manager_->latest();
        const Pose delta = k.frame.odom.inverse() * f.odom;
        const Pose estimated = manager_->graph().estimate(k.id) * delta;
        nav_msgs::msg::Odometry out;
        out.header.stamp = last_stamp_;
        out.header.frame_id =
            output_frame_ + "_s" + std::to_string(manager_->segment());
        out.child_frame_id = sensor_frame_;
        poseMessage(estimated, out.pose.pose);
        out.twist = m.odometry.twist;
        // Move the anchor's right perturbation through the odometry increment
        // and add configured process noise. This is a model approximation.
        {
          Covariance j = Covariance::Zero();
          j.topLeftCorner<3, 3>() = estimated.linear();
          j.bottomRightCorner<3, 3>() = estimated.linear();
          const Covariance world =
              j * manager_->predictionCovariance(f, last_covariance_) *
              j.transpose();
          for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
              out.pose.covariance[r * 6 + c] = world(r, c);
        }
        estimate_pub_->publish(out);
        last_reason_ = f.degraded ? "degraded odometry"
                                  : "odometry only; globally unanchored";
      }
    } catch (const std::exception &e) {
      ++rejected_;
      last_reason_ = e.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Rejected backend input: %s", e.what());
    }
    process_ms_ = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - start)
                      .count();
    process_ms_max_ = std::max(process_ms_max_, process_ms_);
    process_ms_total_ += process_ms_;
  }
  /// 发布输入、轨迹段、图资源、处理时间和存储状态的统计快照。
  /// 时间戳沿用最后接纳帧，计数和最近状态用于旁路监测，不代表已获得全局定位。
  void diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray out;
    out.header.stamp = last_stamp_;
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "rm27_backend";
    s.hardware_id = "point_lio";
    s.level = (graph_failures_ || store_->stats().errors) ? 2 : 0;
    s.message = last_reason_;
    /// 将任意可流式输出的统计值转换为诊断键值项。
    auto put = [&](const char *key, const auto &value) {
      diagnostic_msgs::msg::KeyValue v;
      v.key = key;
      std::ostringstream o;
      o << value;
      v.value = o.str();
      s.values.push_back(v);
    };
    put("received", received_);
    put("accepted", accepted_);
    put("rejected", rejected_);
    put("degraded", degraded_);
    put("segments", segments_);
    put("segment_id", manager_->segment());
    put("sequence_gaps", manager_->sequenceGaps());
    put("keyframes_total", manager_->totalKeyframes());
    put("active_states", manager_->graph().size());
    put("factor_slots", manager_->graph().factorSlots());
    put("marginalized", manager_->graph().marginalized());
    put("graph_failures", graph_failures_);
    put("process_ms", process_ms_);
    put("process_ms_max", process_ms_max_);
    put("process_ms_total", process_ms_total_);
    put("graph_ms", graph_ms_);
    put("graph_ms_max", graph_ms_max_);
    put("cloud_points", last_cloud_points_);
    const auto storage = store_->stats();
    put("cache_bytes", storage.cache_bytes);
    put("cache_clouds", storage.cache_clouds);
    put("cloud_evictions", storage.evictions);
    put("archive_queued", storage.queued);
    put("archive_saved", storage.saved);
    put("archive_dropped", storage.dropped);
    put("archive_errors", storage.errors);
    put("archive_error", storage.last_error);
    put("global_anchor", false);
    put("publishes_tf", false);
    out.status.push_back(s);
    diagnostic_pub_->publish(out);
  }
  std::string output_frame_, sensor_frame_,
      last_reason_ = "waiting for atomic frames";
  std::unique_ptr<tbb::global_control> threads_;
  std::unique_ptr<PointlioAdapter> adapter_;
  std::unique_ptr<KeyframeManager> manager_;
  std::unique_ptr<CloudStore> store_;
  rclcpp::Subscription<PointlioAdapter::Message>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimate_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  builtin_interfaces::msg::Time last_stamp_;
  Covariance last_covariance_ = Covariance::Identity();
  std::uint64_t received_ = 0, accepted_ = 0, rejected_ = 0, degraded_ = 0,
                segments_ = 0, graph_failures_ = 0;
  std::size_t last_cloud_points_ = 0;
  double process_ms_ = 0, process_ms_max_ = 0, process_ms_total_ = 0,
         graph_ms_ = 0, graph_ms_max_ = 0;
};
} // namespace rm27_backend
/// 初始化 ROS，运行后端单线程执行器；退出时先释放节点及写盘线程，再关闭 ROS。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rm27_backend::BackendNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rm27_backend"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
