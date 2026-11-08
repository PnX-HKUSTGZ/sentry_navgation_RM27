#include "rm27_backend/cloud_store.hpp"
#include "rm27_backend/keyframe_manager.hpp"
#include "rm27_backend/loop_closer.hpp"
#include "rm27_backend/map_localizer.hpp"
#include "rm27_backend/pointlio_adapter.hpp"
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <sstream>
#include <tbb/global_control.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

namespace rm27_backend {
/// 装配后端 ROS 输入、关键帧/图管理、先验地图因子、存储与旁路输出的节点。
/// 由单线程执行器串行写图；地图配准和磁盘归档各在独立线程完成，不广播导航 TF。
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
    sensor_world_frame_ = a.world_frame;
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
    // 全局约束三选一，照搬 z-localization：先验地图与回环互斥，回环模式使用全图。
    constraint_mode_ =
        declare_parameter("constraint.mode", std::string("odometry"));
    if (constraint_mode_ != "odometry" && constraint_mode_ != "prior_map" &&
        constraint_mode_ != "loop")
      throw std::invalid_argument(
          "constraint.mode must be odometry, prior_map or loop");
    const bool map_enabled = constraint_mode_ == "prior_map";
    const bool loop_enabled = constraint_mode_ == "loop";
    const auto map_file = declare_parameter("prior_map.file", std::string(""));
    const auto map_transform = declare_parameter(
        "prior_map.transform", std::vector<double>(6, 0.));
    map_frame_ = declare_parameter("prior_map.map_frame", std::string("map"));
    seed_source_ = declare_parameter("prior_map.seed_source", std::string("tf"));
    initial_alignment_ = poseFromVector(declare_parameter(
        "prior_map.initial_alignment", std::vector<double>(6, 0.)));
    MatcherConfig mc;
    mc.map_voxel = declare_parameter("prior_map.voxel_size", mc.map_voxel);
    mc.source_voxel =
        declare_parameter("prior_map.source_voxel_size", mc.source_voxel);
    mc.max_iterations = int(positive("prior_map.max_iterations", mc.max_iterations));
    mc.max_correspondence_distance = declare_parameter(
        "prior_map.max_correspondence_distance", mc.max_correspondence_distance);
    mc.min_inliers = int(positive("prior_map.min_inliers", mc.min_inliers));
    mc.min_inlier_ratio =
        declare_parameter("prior_map.min_inlier_ratio", mc.min_inlier_ratio);
    mc.max_rmse = declare_parameter("prior_map.max_rmse", mc.max_rmse);
    mc.min_coverage_sectors = int(declare_parameter<std::int64_t>(
        "prior_map.min_coverage_sectors", mc.min_coverage_sectors));
    mc.residual_std_floor =
        declare_parameter("prior_map.residual_std_floor", mc.residual_std_floor);
    mc.information_scale =
        declare_parameter("prior_map.information_scale", mc.information_scale);
    mc.translation_std_floor = declare_parameter(
        "prior_map.translation_std_floor", mc.translation_std_floor);
    mc.rotation_std_floor =
        declare_parameter("prior_map.rotation_std_floor_deg", .1) * M_PI / 180.;
    mc.max_condition =
        declare_parameter("prior_map.max_condition", mc.max_condition);
    LocalizerConfig lc;
    lc.correction_period =
        declare_parameter("prior_map.correction_period", lc.correction_period);
    lc.acquisition_period =
        declare_parameter("prior_map.acquisition_period", lc.acquisition_period);
    lc.source_keyframes =
        int(positive("prior_map.source_keyframes", lc.source_keyframes));
    lc.source_max_distance = declare_parameter("prior_map.source_max_distance",
                                               lc.source_max_distance);
    lc.max_translation_correction = declare_parameter(
        "prior_map.max_translation_correction", lc.max_translation_correction);
    lc.max_rotation_correction =
        declare_parameter("prior_map.max_rotation_correction_deg", 15.) * M_PI / 180.;
    lc.acquisition_max_translation = declare_parameter(
        "prior_map.acquisition_max_translation", lc.acquisition_max_translation);
    lc.acquisition_max_rotation =
        declare_parameter("prior_map.acquisition_max_rotation_deg", 20.) * M_PI / 180.;
    lc.rejection_probability = declare_parameter(
        "prior_map.rejection_probability", lc.rejection_probability);
    lc.max_conflicts = int(positive("prior_map.max_conflicts", lc.max_conflicts));
    lc.continuity_timeout =
        declare_parameter("prior_map.continuity_timeout", lc.continuity_timeout);
    source_keyframes_ = lc.source_keyframes;
    source_max_distance_ = lc.source_max_distance;
    ScanContextConfig sc;
    sc.num_rings = int(positive("scan_context.num_rings", sc.num_rings));
    sc.num_sectors = int(positive("scan_context.num_sectors", sc.num_sectors));
    sc.max_radius = declare_parameter("scan_context.max_radius", sc.max_radius);
    sc.downsample = declare_parameter("scan_context.downsample", sc.downsample);
    sc.lidar_height = declare_parameter("scan_context.lidar_height", sc.lidar_height);
    sc.augment_count = int(positive("scan_context.augment_count", sc.augment_count));
    sc.search_ratio = declare_parameter("scan_context.search_ratio", sc.search_ratio);
    sc.lateral_shift = declare_parameter("scan_context.lateral_shift", sc.lateral_shift);
    sc.exclude_recent = int(positive("scan_context.exclude_recent", sc.exclude_recent));
    sc.candidates = int(positive("scan_context.candidates", sc.candidates));
    sc.distance_threshold =
        declare_parameter("scan_context.distance_threshold", sc.distance_threshold);
    LoopConfig loop;
    loop.target_keyframes = int(positive("loop.target_keyframes", loop.target_keyframes));
    loop.target_radius = declare_parameter("loop.target_radius", loop.target_radius);
    loop.target_same_pass =
        declare_parameter("loop.target_same_pass", loop.target_same_pass);
    loop.cooldown_keyframes = int(declare_parameter<std::int64_t>(
        "loop.cooldown_keyframes", loop.cooldown_keyframes));
    loop.query_distance = declare_parameter("loop.query_distance", loop.query_distance);
    loop.query_interval = declare_parameter("loop.query_interval", loop.query_interval);
    const double loop_keyframe_interval =
        declare_parameter("loop.keyframe_max_interval", k.max_interval);
    loop.candidate_max_distance =
        declare_parameter("loop.candidate_max_distance", loop.candidate_max_distance);
    loop.source_keyframes = int(positive("loop.source_keyframes", loop.source_keyframes));
    loop.cloud_voxel = declare_parameter("loop.voxel_size", loop.cloud_voxel);
    loop.threads = int(declare_parameter<std::int64_t>("loop.threads", loop.threads));
    loop.max_keyframes = positive("loop.max_keyframes", loop.max_keyframes);
    loop.history_size = positive("loop.history_size", loop.history_size);
    loop.max_translation = declare_parameter("loop.max_translation", loop.max_translation);
    loop.max_z = declare_parameter("loop.max_z", loop.max_z);
    loop.max_roll_pitch = declare_parameter("loop.max_roll_pitch_deg", 10.) * M_PI / 180.;
    loop.max_yaw_residual =
        declare_parameter("loop.max_yaw_residual_deg", 60.) * M_PI / 180.;
    loop.min_overlap = declare_parameter("loop.min_overlap_ratio", loop.min_overlap);
    loop.pcm = declare_parameter("loop.pcm_enabled", loop.pcm);
    loop.pcm_translation =
        declare_parameter("loop.pcm_translation_threshold", loop.pcm_translation);
    loop.pcm_rotation =
        declare_parameter("loop.pcm_rotation_threshold_deg", 15.) * M_PI / 180.;
    loop.rejection_probability =
        declare_parameter("loop.rejection_probability", loop.rejection_probability);
    // 回环配准门限照搬 z-localization 的 registration 配置。
    MatcherConfig lm;
    lm.map_voxel = lm.source_voxel = loop.cloud_voxel;
    lm.min_inliers = int(positive("loop.min_correspondences", 80));
    lm.min_inlier_ratio = declare_parameter("loop.min_correspondence_ratio", .5);
    lm.max_rmse = declare_parameter("loop.max_rmse", .25);
    lm.min_coverage_sectors = int(declare_parameter<std::int64_t>("loop.min_coverage_sectors", 6));
    lm.translation_std_floor = declare_parameter("loop.translation_std_floor", .005);
    lm.rotation_std_floor =
        declare_parameter("loop.rotation_std_floor_deg", .02) * M_PI / 180.;
    lm.overlap_distance = declare_parameter("loop.overlap_distance", lm.overlap_distance);
    g.loop_cauchy_size = declare_parameter("graph.loop_cauchy_size", .5);
    if (loop_enabled) {
      // 回环需要历史节点仍在图中：关闭时间滞后，仅以回环库上限约束节点数。
      g.lag_seconds = 0;
      g.max_states = loop.max_keyframes;
      // 全图不靠定时关键帧维持窗口；静止时少建帧，节点、回环库和图更新开销随之下降。
      k.max_interval = loop_keyframe_interval;
    }
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
    if (loop_enabled)
      loop_ = std::make_unique<LoopCloser>(loop, sc, lm);
    if (map_enabled) {
      if (map_transform.size() != 6 ||
          (seed_source_ != "tf" && seed_source_ != "parameter"))
        throw std::invalid_argument("invalid prior_map transform or seed_source");
      // 地图在后台线程加载，失败时节点退回纯里程计图并在诊断中报错。
      localizer_ = std::make_unique<MapLocalizer>(lc, mc, map_file,
                                                  poseFromVector(map_transform));
      if (seed_source_ == "tf") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      }
      alignment_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
          "~/map_alignment", 10);
    }
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
    RCLCPP_INFO(get_logger(),
                "iSAM2 backend ready; constraint=%s, prior_map=%s, output=%s, no TF; archive=%s",
                constraint_mode_.c_str(), map_enabled ? map_file.c_str() : "disabled",
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
  /// 地图因子是否在本段生效：定位器已加载地图；加载失败时退回纯里程计图。
  bool mapMode() const { return localizer_ && localizer_->ready(); }
  /// 当前输出坐标系：已锚定时为 map，否则为带段号的局部图坐标系。
  std::string graphFrame() const {
    if (mapMode() && localizer_->anchored())
      return map_frame_;
    return output_frame_ + "_s" + std::to_string(manager_->segment());
  }
  /// 生成本帧的 map←IMU 锚定种子：优先断段连续性，其次 TF 或参数给出的 map←world 对齐。
  std::optional<Pose> seed(const Frame &f,
                           const builtin_interfaces::msg::Time &stamp) {
    if (auto continuity = localizer_->continuitySeed(f.stamp_ns))
      return continuity;
    if (seed_source_ == "parameter")
      return initial_alignment_ * f.odom;
    // map←world 由导航定位链提供，仅作为锚定初值，配准仍需满秩和种子偏差验收。
    // 优先取同刻变换；定位链发布稍滞后时退回最新变换，偏差由验收门限约束。
    for (const auto &time : {rclcpp::Time(stamp), rclcpp::Time(0, 0, get_clock()->get_clock_type())}) {
      try {
        const auto t = tf_buffer_->lookupTransform(map_frame_, sensor_world_frame_, time);
        return Pose(tf2::transformToEigen(t)) * f.odom;
      } catch (const std::exception &) {
      }
    }
    return std::nullopt;
  }
  /// 以当前关键帧为参考，用原始里程计相对位姿合并近邻关键帧点云作为跟踪源。
  /// 只读取缓存或归档中仍存在的点云，缺失帧直接跳过。
  std::pair<Cloud::ConstPtr, int> trackingSource(const Keyframe &k,
                                                 Cloud::ConstPtr current) {
    auto merged = std::make_shared<Cloud>(*current);
    int frames = 1;
    const Pose reference = k.frame.odom.inverse();
    const auto &active = manager_->keyframes();
    for (auto it = active.rbegin();
         it != active.rend() && frames < source_keyframes_; ++it) {
      const auto &other = it->second;
      if (other.id >= k.id || other.frame.identity != k.frame.identity)
        continue;
      const Pose relative = reference * other.frame.odom;
      if (relative.translation().norm() > source_max_distance_)
        break;
      Cloud::ConstPtr cloud;
      try {
        cloud = store_->get(other.id);
      } catch (const std::exception &) {
      }
      if (!cloud)
        continue;
      Cloud moved;
      pcl::transformPointCloud(*cloud, moved, relative.matrix().cast<float>());
      *merged += moved;
      ++frames;
    }
    return {merged, frames};
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
      if (admission == Admission::NewSegment &&
          manager_->segment() != counted_segment_) {
        // 未建图时每帧都会报告新段，只按段号变化计数和重置地图定位。
        ++segments_;
        counted_segment_ = manager_->segment();
        last_reason_ = manager_->reason();
      }
      if (localizer_)
        localizer_->syncSegment(manager_->segment());
      if (loop_)
        loop_->syncSegment(manager_->segment());
      last_stamp_ = m.odometry.header.stamp;
      ++accepted_;
      if (f.degraded)
        ++degraded_;
      // 先提交后台已完成的地图配准，再处理本帧，保证图写入始终在本线程串行。
      if (mapMode()) {
        const auto outcome = localizer_->apply(*manager_);
        if (outcome.graph_failure) {
          manager_->breakSegment();
          localizer_->resetSegment();
          ++graph_failures_;
        } else if (outcome.accepted && manager_->latest()) {
          last_covariance_ = manager_->graph().covariance(manager_->latest()->id);
        }
      }
      // 提交后台已验证的回环；图异常时断段并清空回环库。
      if (loop_) {
        const auto outcome = loop_->apply(*manager_);
        for (const auto &r : loop_->takeAccepted()) {
          // 单行记录便于离线按时间戳与真值比对回环测量误差。
          const auto &t = r.measurement.relative.translation();
          const Eigen::Quaterniond q(r.measurement.relative.linear());
          RCLCPP_INFO(get_logger(),
                      "loop accepted %s from_ns=%ld to_ns=%ld "
                      "rel=%.5f,%.5f,%.5f,%.7f,%.7f,%.7f,%.7f nis=%.3f rmse=%.4f overlap=%.3f",
                      r.measurement.measurement_id.c_str(), long(r.from_stamp_ns),
                      long(r.to_stamp_ns), t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w(),
                      r.nis, r.rmse, r.overlap);
        }
        if (outcome.graph_failure) {
          manager_->breakSegment();
          loop_->syncSegment(manager_->segment());
          ++graph_failures_;
        } else if (outcome.accepted && manager_->latest()) {
          last_covariance_ = manager_->graph().covariance(manager_->latest()->id);
        }
      }
      // 地图仍在加载时不建图，避免本段先在 odom 系建立后无法切换到 map 系。
      const bool waiting_map =
          localizer_ && localizer_->state() == MapState::Loading;
      if (!waiting_map && manager_->shouldCreate(f)) {
        // 先准备好有效点云，再提交对应的关键帧与图状态。
        auto cloud = adapter_->cloud(m, f);
        std::optional<PosePrior> prior;
        if (mapMode() && !manager_->latest()) {
          prior = localizer_->takeAnchor(f);
          if (!prior) {
            // 未锚定前不建图：用本帧点云和种子请求一次满秩配准。
            if (const auto initial = seed(f, m.odometry.header.stamp))
              localizer_->requestAcquisition(f, cloud, *initial);
            last_reason_ = localizer_->reason();
            finishTiming(start);
            return;
          }
        }
        const auto graph_start = std::chrono::steady_clock::now();
        Keyframe k;
        try {
          k = manager_->commit(f, prior);
        } catch (...) {
          manager_->breakSegment();
          if (localizer_)
            localizer_->resetSegment();
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
        path.header.frame_id = graphFrame();
        // 关键帧估计已在提交时整体同步，直接读取元数据，避免逐节点查询图。
        path.poses.reserve(manager_->keyframes().size());
        for (const auto &[id, keyframe] : manager_->keyframes()) {
          geometry_msgs::msg::PoseStamped item;
          item.header = path.header;
          item.header.stamp = rclcpp::Time(keyframe.frame.stamp_ns);
          poseMessage(keyframe.estimate, item.pose);
          path.poses.push_back(item);
        }
        path_pub_->publish(path);
        last_covariance_ = manager_->graph().covariance(k.id);
        // 非首帧关键帧按校正周期排队跟踪配准；初值取图估计。
        if (mapMode() && localizer_->anchored() && manager_->lastMeasurement()) {
          const auto [source, frames] = trackingSource(k, cloud);
          localizer_->requestTracking(k, k.estimate, source, frames);
        }
        // 回环：登记描述子与降采样点云，再为最新关键帧检索并排队验证。
        if (loop_) {
          loop_->addKeyframe(k, *cloud);
          loop_->request(*manager_);
        }
      }
      if (manager_->latest()) {
        const auto &k = *manager_->latest();
        const Pose delta = k.frame.odom.inverse() * f.odom;
        // 最新关键帧估计在每次图更新后已同步，无需再次回代。
        const Pose estimated = k.estimate * delta;
        nav_msgs::msg::Odometry out;
        out.header.stamp = last_stamp_;
        out.header.frame_id = graphFrame();
        out.child_frame_id = sensor_frame_;
        poseMessage(estimated, out.pose.pose);
        out.twist = m.odometry.twist;
        // 通过里程计增量传播锚点的右扰动协方差，
        // 并加入配置的过程噪声；该结果属于模型近似。
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
        if (mapMode() && localizer_->anchored()) {
          // 发布 map←前端世界系对齐，等价于后续可接管的 map→odom 估计，但不广播 TF。
          localizer_->rememberPose(estimated, f.stamp_ns);
          geometry_msgs::msg::TransformStamped alignment;
          alignment.header.stamp = last_stamp_;
          alignment.header.frame_id = map_frame_;
          alignment.child_frame_id = sensor_world_frame_;
          alignment.transform =
              tf2::eigenToTransform(Eigen::Isometry3d(estimated * f.odom.inverse()))
                  .transform;
          alignment_pub_->publish(alignment);
          last_reason_ = f.degraded ? "degraded odometry; map anchored"
                                    : localizer_->reason();
        } else {
          last_reason_ = f.degraded ? "degraded odometry"
                                    : "odometry only; globally unanchored";
        }
      }
    } catch (const std::exception &e) {
      ++rejected_;
      last_reason_ = e.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Rejected backend input: %s", e.what());
    }
    finishTiming(start);
  }
  /// 记录单帧处理耗时、最大值和累计值。
  void finishTiming(std::chrono::steady_clock::time_point start) {
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
    const bool anchored = mapMode() && localizer_->anchored();
    put("global_anchor", anchored);
    put("output_frame", graphFrame());
    put("constraint_mode", constraint_mode_);
    if (loop_) {
      const auto l = loop_->stats();
      put("loop_reason", loop_->reason());
      put("loop_keyframes", l.keyframes);
      put("loop_descriptor_bytes", l.descriptor_bytes);
      put("loop_cloud_bytes", l.cloud_bytes);
      put("loop_queries", l.queries);
      put("loop_candidates", l.candidates);
      put("loop_verified", l.verified);
      put("loop_accepted", l.accepted);
      put("loop_registration_rejects", l.registration_rejects);
      put("loop_geometry_rejects", l.geometry_rejects);
      put("loop_pcm_rejects", l.pcm_rejects);
      put("loop_nis_rejects", l.nis_rejects);
      put("loop_busy_skips", l.busy_skips);
      put("loop_stale", l.stale);
      put("loop_last_nis", l.last_nis);
      put("loop_last_rmse", l.last_rmse);
      put("loop_last_overlap", l.last_overlap);
      put("loop_last_edge", std::to_string(l.last_from) + "->" + std::to_string(l.last_to));
      put("loop_verify_ms", l.last_ms);
      put("loop_verify_ms_max", l.max_ms);
      put("loop_verify_ms_total", l.total_ms);
      put("loop_nis_ms_max", l.nis_ms_max);
      put("loop_update_ms_max", l.update_ms_max);
    }
    if (localizer_) {
      const auto m = localizer_->stats();
      if (localizer_->state() == MapState::Failed)
        s.level = 2;
      put("map_state", toString(localizer_->state()));
      put("map_reason", localizer_->reason());
      put("map_points", m.map_points);
      put("map_planar_points", m.planar_points);
      put("map_requests", m.requests);
      put("map_completed", m.completed);
      put("map_busy_skips", m.busy_skips);
      put("map_stale", m.stale);
      put("map_anchors", m.anchors);
      put("map_acquisition_failures", m.acquisition_failures);
      put("map_accepted", m.accepted);
      put("map_quality_rejects", m.quality_rejects);
      put("map_correction_rejects", m.correction_rejects);
      put("map_nis_rejects", m.nis_rejects);
      put("map_recoveries", m.recoveries);
      put("map_consecutive_conflicts", m.consecutive_conflicts);
      put("map_last_rmse", m.last_rmse);
      put("map_last_inlier_ratio", m.last_inlier_ratio);
      put("map_last_rank", m.last_rank);
      put("map_last_nis", m.last_nis);
      put("map_last_correction_m", m.last_correction);
      put("map_registration_ms", m.last_ms);
      put("map_registration_ms_max", m.max_ms);
      put("map_registration_ms_total", m.total_ms);
    }
    put("publishes_tf", false);
    out.status.push_back(s);
    diagnostic_pub_->publish(out);
  }
  std::string map_frame_, seed_source_, sensor_world_frame_;
  Pose initial_alignment_ = Pose::Identity();
  int source_keyframes_ = 3;
  double source_max_distance_ = 1.5;
  std::unique_ptr<MapLocalizer> localizer_;
  std::unique_ptr<LoopCloser> loop_;
  std::string constraint_mode_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
      alignment_pub_;
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
  std::uint64_t counted_segment_ = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t received_ = 0, accepted_ = 0, rejected_ = 0, degraded_ = 0,
                segments_ = 0, graph_failures_ = 0;
  std::size_t last_cloud_points_ = 0;
  double process_ms_ = 0, process_ms_max_ = 0, process_ms_total_ = 0,
         graph_ms_ = 0, graph_ms_max_ = 0;
};
} // 命名空间 rm27_backend
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
