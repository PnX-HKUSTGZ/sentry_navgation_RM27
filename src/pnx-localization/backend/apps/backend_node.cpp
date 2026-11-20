#include "rm27_backend/cloud_store.hpp"
#include "rm27_backend/keyframe_manager.hpp"
#include "rm27_backend/localization_controller.hpp"
#include "rm27_backend/loop_closer.hpp"
#include "rm27_backend/navigation_output_queue.hpp"
#include "rm27_backend/pointlio_adapter.hpp"
#include <action_msgs/srv/cancel_goal.hpp>
#include <algorithm>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <limits>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <rm27_localization_interfaces/msg/localization_status.hpp>
#include <rm27_localization_interfaces/srv/reset_frontend.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <tbb/global_control.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rm27_backend {
/// 装配后端 ROS 输入、关键帧/图管理、先验地图因子、存储与旁路输出的节点。
/// 由单线程执行器串行写图；地图配准和磁盘归档各在独立线程完成，由统一可信输出发布导航
/// TF。
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
    // 全局约束三选一，照搬
    // z-localization：先验地图与回环互斥，回环模式使用全图。
    constraint_mode_ =
        declare_parameter("constraint.mode", std::string("odometry"));
    if (constraint_mode_ != "odometry" && constraint_mode_ != "prior_map" &&
        constraint_mode_ != "loop")
      throw std::invalid_argument(
          "constraint.mode must be odometry, prior_map or loop");
    const bool map_enabled = constraint_mode_ == "prior_map";
    const bool loop_enabled = constraint_mode_ == "loop";
    const auto map_file = declare_parameter("prior_map.file", std::string(""));
    const auto map_transform =
        declare_parameter("prior_map.transform", std::vector<double>(6, 0.));
    map_frame_ = declare_parameter("prior_map.map_frame", std::string("map"));
    seed_source_ =
        declare_parameter("prior_map.seed_source", std::string("none"));
    initial_alignment_ = poseFromVector(declare_parameter(
        "prior_map.initial_alignment", std::vector<double>(6, 0.)));
    startup_pose_enabled_ = declare_parameter("startup_pose.enabled", false);
    startup_map_base_ = poseFromVector(declare_parameter(
        "startup_pose.map_from_base", std::vector<double>(6, 0.)));
    if (startup_pose_enabled_ && seed_source_ != "none")
      throw std::invalid_argument(
          "choose startup_pose or initial_alignment, not both");
    MatcherConfig mc;
    mc.gravity_aligned = declare_parameter("prior_map.gravity_aligned", true);
    mc.map_voxel = declare_parameter("prior_map.voxel_size", mc.map_voxel);
    mc.source_voxel =
        declare_parameter("prior_map.source_voxel_size", mc.source_voxel);
    mc.max_iterations =
        int(positive("prior_map.max_iterations", mc.max_iterations));
    mc.max_correspondence_distance =
        declare_parameter("prior_map.max_correspondence_distance",
                          mc.max_correspondence_distance);
    mc.min_inliers = int(positive("prior_map.min_inliers", mc.min_inliers));
    mc.min_inlier_ratio =
        declare_parameter("prior_map.min_inlier_ratio", mc.min_inlier_ratio);
    mc.max_rmse = declare_parameter("prior_map.max_rmse", mc.max_rmse);
    mc.min_coverage_sectors = int(declare_parameter<std::int64_t>(
        "prior_map.min_coverage_sectors", mc.min_coverage_sectors));
    mc.residual_std_floor = declare_parameter("prior_map.residual_std_floor",
                                              mc.residual_std_floor);
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
    lc.acquisition_period = declare_parameter("prior_map.acquisition_period",
                                              lc.acquisition_period);
    lc.source_keyframes =
        int(positive("prior_map.source_keyframes", lc.source_keyframes));
    lc.source_max_distance = declare_parameter("prior_map.source_max_distance",
                                               lc.source_max_distance);
    lc.max_translation_correction = declare_parameter(
        "prior_map.max_translation_correction", lc.max_translation_correction);
    lc.max_rotation_correction =
        declare_parameter("prior_map.max_rotation_correction_deg", 15.) * M_PI /
        180.;
    lc.acquisition_max_translation =
        declare_parameter("prior_map.acquisition_max_translation",
                          lc.acquisition_max_translation);
    lc.acquisition_max_rotation =
        declare_parameter("prior_map.acquisition_max_rotation_deg", 20.) *
        M_PI / 180.;
    lc.rejection_probability = declare_parameter(
        "prior_map.rejection_probability", lc.rejection_probability);
    lc.max_conflicts =
        int(positive("prior_map.max_conflicts", lc.max_conflicts));
    lc.continuity_timeout = declare_parameter("prior_map.continuity_timeout",
                                              lc.continuity_timeout);
    source_keyframes_ = lc.source_keyframes;
    source_max_distance_ = lc.source_max_distance;
    ScanContextConfig sc;
    sc.num_rings = int(positive("scan_context.num_rings", sc.num_rings));
    sc.num_sectors = int(positive("scan_context.num_sectors", sc.num_sectors));
    sc.max_radius = declare_parameter("scan_context.max_radius", sc.max_radius);
    sc.downsample = declare_parameter("scan_context.downsample", sc.downsample);
    sc.lidar_height =
        declare_parameter("scan_context.lidar_height", sc.lidar_height);
    sc.augment_count =
        int(positive("scan_context.augment_count", sc.augment_count));
    sc.search_ratio =
        declare_parameter("scan_context.search_ratio", sc.search_ratio);
    sc.lateral_shift =
        declare_parameter("scan_context.lateral_shift", sc.lateral_shift);
    sc.exclude_recent =
        int(positive("scan_context.exclude_recent", sc.exclude_recent));
    sc.candidates = int(positive("scan_context.candidates", sc.candidates));
    sc.distance_threshold = declare_parameter("scan_context.distance_threshold",
                                              sc.distance_threshold);
    LoopConfig loop;
    loop.target_keyframes =
        int(positive("loop.target_keyframes", loop.target_keyframes));
    loop.target_radius =
        declare_parameter("loop.target_radius", loop.target_radius);
    loop.target_same_pass =
        declare_parameter("loop.target_same_pass", loop.target_same_pass);
    loop.cooldown_keyframes = int(declare_parameter<std::int64_t>(
        "loop.cooldown_keyframes", loop.cooldown_keyframes));
    loop.query_distance =
        declare_parameter("loop.query_distance", loop.query_distance);
    loop.query_interval =
        declare_parameter("loop.query_interval", loop.query_interval);
    const double loop_keyframe_interval =
        declare_parameter("loop.keyframe_max_interval", k.max_interval);
    loop.candidate_max_distance = declare_parameter(
        "loop.candidate_max_distance", loop.candidate_max_distance);
    loop.source_keyframes =
        int(positive("loop.source_keyframes", loop.source_keyframes));
    loop.cloud_voxel = declare_parameter("loop.voxel_size", loop.cloud_voxel);
    loop.threads =
        int(declare_parameter<std::int64_t>("loop.threads", loop.threads));
    loop.max_keyframes = positive("loop.max_keyframes", loop.max_keyframes);
    loop.history_size = positive("loop.history_size", loop.history_size);
    loop.max_translation =
        declare_parameter("loop.max_translation", loop.max_translation);
    loop.max_z = declare_parameter("loop.max_z", loop.max_z);
    loop.max_roll_pitch =
        declare_parameter("loop.max_roll_pitch_deg", 10.) * M_PI / 180.;
    loop.max_yaw_residual =
        declare_parameter("loop.max_yaw_residual_deg", 60.) * M_PI / 180.;
    loop.min_overlap =
        declare_parameter("loop.min_overlap_ratio", loop.min_overlap);
    loop.pcm = declare_parameter("loop.pcm_enabled", loop.pcm);
    loop.pcm_translation = declare_parameter("loop.pcm_translation_threshold",
                                             loop.pcm_translation);
    loop.pcm_rotation =
        declare_parameter("loop.pcm_rotation_threshold_deg", 15.) * M_PI / 180.;
    loop.rejection_probability = declare_parameter("loop.rejection_probability",
                                                   loop.rejection_probability);
    // 回环配准门限照搬 z-localization 的 registration 配置。
    MatcherConfig lm;
    lm.map_voxel = lm.source_voxel = loop.cloud_voxel;
    lm.min_inliers = int(positive("loop.min_correspondences", 80));
    lm.min_inlier_ratio =
        declare_parameter("loop.min_correspondence_ratio", .5);
    lm.max_rmse = declare_parameter("loop.max_rmse", .25);
    lm.min_coverage_sectors =
        int(declare_parameter<std::int64_t>("loop.min_coverage_sectors", 6));
    lm.translation_std_floor =
        declare_parameter("loop.translation_std_floor", .005);
    lm.rotation_std_floor =
        declare_parameter("loop.rotation_std_floor_deg", .02) * M_PI / 180.;
    lm.overlap_distance =
        declare_parameter("loop.overlap_distance", lm.overlap_distance);
    g.loop_cauchy_size = declare_parameter("graph.loop_cauchy_size", .5);
    if (loop_enabled) {
      // 回环需要历史节点仍在图中：关闭时间滞后，仅以回环库上限约束节点数。
      g.lag_seconds = 0;
      g.max_states = loop.max_keyframes;
      // 全图不靠定时关键帧维持窗口；静止时少建帧，节点、回环库和图更新开销随之下降。
      k.max_interval = loop_keyframe_interval;
    }
    ControllerConfig cc;
    cc.tracking = lc;
    cc.rmse_failures_before_suspect =
        int(positive("prior_map.rmse_failures_before_suspect", 3));
    cc.allow_unbounded_auto_recovery =
        declare_parameter("recovery.allow_unbounded_auto_recovery", false);
    cc.degraded_frames_before_suspect =
        int(positive("state.degraded_frames_before_suspect", 10));
    cc.recovery.source_pcd = map_file;
    cc.recovery.navigation_grid =
        declare_parameter("recovery.navigation_grid", std::string(""));
    cc.source_min_frames = int(positive("recovery.source_min_frames", 2));
    cc.source_max_frames = int(positive("recovery.source_max_frames", 5));
    cc.source_span = declare_parameter("recovery.source_span", .5);
    cc.result_max_age = declare_parameter("recovery.result_max_age", 8.);
    cc.input_timeout = declare_parameter("state.input_timeout", 1.);
    cc.failures_before_lost =
        int(positive("state.recovery_failures_before_lost", 3));
    cc.recovery.candidates = int(positive("recovery.candidates", 8));
    cc.recovery.yaw_samples = int(positive("recovery.yaw_samples", 6));
    cc.recovery.cache_targets = int(positive("recovery.cache_targets", 8));
    cc.recovery.suspect_radius =
        declare_parameter("recovery.suspect_radius", 8.);
    cc.recovery.target_radius =
        declare_parameter("recovery.target_radius", 12.);
    cc.recovery.max_seconds = declare_parameter("recovery.max_seconds", 5.);
    cc.recovery.ambiguity_ratio =
        declare_parameter("recovery.ambiguity_ratio", .9);
    cc.recovery.ambiguity_angle_rad =
        declare_parameter("recovery.ambiguity_angle_deg", 30.) * M_PI / 180.;
    cc.recovery.ambiguity_distance =
        declare_parameter("recovery.ambiguity_distance", 2.);
    cc.recovery.min_overlap = declare_parameter("recovery.min_overlap", .45);
    cc.recovery.hint_global_fallback =
        declare_parameter("recovery.hint_global_fallback", true);
    cc.recovery.hint_yaw_samples =
        int(positive("recovery.hint_yaw_samples", 3));
    cc.recovery.hint_yaw_step =
        declare_parameter("recovery.hint_yaw_step_deg", 10.) * M_PI / 180.;
    cc.ambiguity_retry_period =
        declare_parameter("recovery.ambiguity_retry_period", 3.);
    cc.motion.enabled = declare_parameter("recovery.motion.enabled", true);
    cc.motion.max_age = declare_parameter("recovery.motion.max_age", 15.);
    cc.motion.base_radius =
        declare_parameter("recovery.motion.base_radius", 1.5);
    cc.motion.max_radius = declare_parameter("recovery.motion.max_radius", 8.);
    cc.motion.drift_per_second =
        declare_parameter("recovery.motion.drift_per_second", .15);
    cc.motion.drift_per_meter =
        declare_parameter("recovery.motion.drift_per_meter", .08);
    cc.motion.base_angle =
        declare_parameter("recovery.motion.base_angle_deg", 15.) * M_PI / 180.;
    cc.motion.max_angle =
        declare_parameter("recovery.motion.max_angle_deg", 60.) * M_PI / 180.;
    cc.motion.angle_per_second =
        declare_parameter("recovery.motion.angle_per_second_deg", 1.) * M_PI /
        180.;
    cc.motion.angle_per_meter =
        declare_parameter("recovery.motion.angle_per_meter_deg", 2.) * M_PI /
        180.;
    cc.motion.angle_per_radian =
        declare_parameter("recovery.motion.angle_per_radian", .1);
    cc.motion.max_gap = k.max_gap;
    cc.motion.max_speed =
        std::min(k.max_linear_speed,
                 declare_parameter("recovery.motion.max_linear_speed", 6.));
    cc.motion.max_angular_speed = std::min(
        k.max_angular_speed,
        declare_parameter("recovery.motion.max_angular_speed_deg", 180.) *
            M_PI / 180.);
    manual_preview_enabled_ = declare_parameter("initialpose.preview_tf", true);
    manual_timeout_ = declare_parameter("initialpose.timeout", 5.);
    manual_tf_timeout_ = declare_parameter("initialpose.tf_wait_timeout", 1.);
    hint_max_translation_ =
        declare_parameter("initialpose.max_translation", 2.);
    hint_max_rotation_ =
        declare_parameter("initialpose.max_rotation_deg", 60.) * M_PI / 180.;
    for (double value : {manual_timeout_, manual_tf_timeout_,
                         hint_max_translation_, hint_max_rotation_})
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("invalid initialpose limits");
    if (hint_max_translation_ < .5 || hint_max_rotation_ < .1745329252)
      throw std::invalid_argument("initialpose maximum bounds below minimum");
    bundle_ = declare_parameter("recovery.map_bundle", std::string(""));
    if (bundle_.empty())
      bundle_ = map_file + ".rm27";
    publish_tf_ = declare_parameter("output.publish_tf", false);
    odom_frame_ = declare_parameter("output.odom_frame", std::string("odom"));
    base_frame_ =
        declare_parameter("output.base_frame", std::string("base_footprint"));
    input_timeout_ = cc.input_timeout;
    auto_reset_ = declare_parameter("state.reset_on_frontend_timeout", true);
    reset_timeout_ = declare_parameter("state.reset_timeout", 5.);
    if (!std::isfinite(reset_timeout_) || reset_timeout_ <= 0)
      throw std::invalid_argument("invalid reset timeout");

    if (publish_tf_ && !map_enabled)
      throw std::invalid_argument("navigation TF requires prior_map mode");
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
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (publish_tf_)
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (map_enabled) {
      if (seed_source_ != "none" && seed_source_ != "parameter")
        throw std::invalid_argument(
            "prior_map.seed_source must be none or parameter; TF seed creates "
            "a startup cycle");
      localizer_ = std::make_unique<LocalizationController>(
          cc, mc, sc, bundle_, poseFromVector(map_transform));
      alignment_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
          "~/map_alignment", 10);
    }
    background_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/prior_pcd_map", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ =
        create_publisher<rm27_localization_interfaces::msg::LocalizationStatus>(
            "~/localization_status",
            rclcpp::QoS(1).reliable().transient_local());
    recovery_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/recovery_markers", rclcpp::QoS(1).reliable().transient_local());
    clearRecoveryMarkers();
    cancel_navigation_ = create_client<action_msgs::srv::CancelGoal>(
        "/navigate_to_pose/_action/cancel_goal");
    clear_global_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/global_costmap/clear_entirely_global_costmap");
    clear_local_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/local_costmap/clear_entirely_local_costmap");
    initial_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
                       m) { initialPose(*m); });
    recovery_service_ = create_service<std_srvs::srv::Trigger>(
        "~/relocalize",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = bool(localizer_);
          response->message = localizer_ ? "global recovery scheduled"
                                         : "prior map mode required";
          if (localizer_) {
            pending_hint_.reset();
            preview_base_.reset();
            manual_active_ = false;
            localizer_->manual(std::nullopt);
            clearPendingNavigation();
          }
          publishStatus();
        });
    reset_client_ =
        create_client<rm27_localization_interfaces::srv::ResetFrontend>(
            "/point_lio/reset");
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_frontend",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = requestReset();
          response->message = response->success
                                  ? "reset queued; waiting for new epoch"
                                  : "frontend unavailable or reset already "
                                    "attempted for this epoch";
        });
    // 与图写入共用串行执行器；只在等待同刻 TF 时启用快速重试。
    navigation_retry_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      flushNavigationOutputs();
      publishStatus();
    });
    navigation_retry_->cancel();
    maintenance_ = create_wall_timer(std::chrono::milliseconds(100),
                                     [this] { maintain(); });
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
                "iSAM2 backend ready; constraint=%s, prior_map=%s, output=%s; "
                "archive=%s",
                constraint_mode_.c_str(),
                map_enabled ? map_file.c_str() : "disabled",
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
  /// 地图已准备好才允许配准提交；加载失败保持未定位，导航许可关闭。
  bool mapMode() const { return localizer_ && localizer_->ready(); }
  /// 当前输出坐标系：已锚定时为 map，否则为带段号的局部图坐标系。
  std::string graphFrame() const {
    if (mapMode() && localizer_->anchored())
      return map_frame_;
    return output_frame_ + "_s" + std::to_string(manager_->segment());
  }
  /// 原子帧的 IMU body 是状态参考点，并不要求存在同名
  /// TF；按源时间插值前端位姿。
  std::optional<Pose> frontendPoseAt(std::int64_t stamp) const {
    for (std::size_t i = 0; i < odom_history_.size(); ++i) {
      const auto &a = odom_history_[i];
      if (a.stamp_ns == stamp)
        return a.odom;
      if (i + 1 >= odom_history_.size())
        break;
      const auto &b = odom_history_[i + 1];
      if (a.stamp_ns < stamp && stamp < b.stamp_ns &&
          a.identity == b.identity) {
        const double fraction =
            double(stamp - a.stamp_ns) / double(b.stamp_ns - a.stamp_ns);
        Pose pose = Pose::Identity();
        pose.translation() = (1 - fraction) * a.odom.translation() +
                             fraction * b.odom.translation();
        pose.linear() =
            Eigen::Quaterniond(a.odom.linear())
                .slerp(fraction, Eigen::Quaterniond(b.odom.linear()))
                .toRotationMatrix();
        return pose;
      }
    }
    return std::nullopt;
  }
  /// 只清理导航代价地图；ROG
  /// 在同刻坐标变换刷新后重新投影，几何验收门限保持不变。
  void clearNavigationMaps() {
    for (const auto &client : {clear_global_, clear_local_})
      if (client && client->service_is_ready())
        client->async_send_request(
            std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
  }
  /// 受理人工初值时取消当前导航目标；恢复后由新的目标生成有效规划会话。
  void cancelNavigation() {
    if (cancel_navigation_->service_is_ready())
      cancel_navigation_->async_send_request(
          std::make_shared<action_msgs::srv::CancelGoal::Request>());
  }
  /// 精确删除本节点的恢复箭头和文字，持久化删除消息供迟到的 RViz 同步。
  void clearRecoveryMarkers() {
    if (!recovery_markers_ || !recovery_markers_visible_)
      return;
    visualization_msgs::msg::MarkerArray array;
    for (int id : {0, 1}) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = now();
      marker.ns = "manual_recovery";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(marker);
    }
    recovery_markers_->publish(array);
    manual_pose_.reset();
    recovery_markers_visible_ = false;
    manual_failed_ = false;
  }
  /// 发布待验证或失败箭头及耗时；恢复成功由删除消息清理。
  void publishRecoveryMarkers() {
    if (!manual_pose_ || !recovery_markers_)
      return;
    visualization_msgs::msg::MarkerArray array;
    recovery_markers_visible_ = true;
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = map_frame_;
    arrow.header.stamp = now();
    arrow.ns = "manual_recovery";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    poseMessage(*manual_pose_, arrow.pose);
    arrow.scale.x = .8;
    arrow.scale.y = .12;
    arrow.scale.z = .12;
    arrow.color.a = 1.;
    arrow.color.r = manual_active_ || manual_failed_ ? 1. : .1;
    arrow.color.g = manual_failed_ ? .1 : .85;
    array.markers.push_back(arrow);
    auto label = arrow;
    label.id = 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.pose.orientation = geometry_msgs::msg::Quaternion{};
    label.pose.orientation.w = 1.;
    label.pose.position.z += 1.;
    label.scale.z = .24;
    std::ostringstream text;
    text << manual_phase_ << " (" << std::fixed << std::setprecision(2)
         << (manual_active_
                 ? std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - manual_started_)
                       .count()
                 : manual_elapsed_)
         << " s)";
    label.text = text.str();
    array.markers.push_back(label);
    recovery_markers_->publish(array);
  }
  /// RViz 初值立即撤销可信许可并显示候选；原始时间戳保留到实际配准初值的转换。
  void initialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &m) {
    if (!localizer_ || !latest_frame_ || m.header.frame_id != map_frame_) {
      RCLCPP_WARN(
          get_logger(),
          "Initial pose rejected: prior-map frontend and map frame required");
      return;
    }
    const auto stamp =
        rclcpp::Time(m.header.stamp, get_clock()->get_clock_type());
    const auto &q = m.pose.pose.orientation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    Pose map_base = Pose::Identity();
    map_base.translation() << m.pose.pose.position.x, m.pose.pose.position.y,
        m.pose.pose.position.z;
    if (stamp.nanoseconds() <= 0 ||
        std::abs((stamp - rclcpp::Time(latest_frame_->stamp_ns,
                                       get_clock()->get_clock_type()))
                     .seconds()) > .5 ||
        !rotation.coeffs().allFinite() ||
        std::abs(rotation.norm() - 1.) > 1e-3 ||
        !map_base.translation().allFinite()) {
      RCLCPP_WARN(get_logger(),
                  "Initial pose rejected: invalid geometry or source time");
      return;
    }
    for (int i : {0, 7, 35})
      if (!std::isfinite(m.pose.covariance[i]) || m.pose.covariance[i] < 0) {
        RCLCPP_WARN(get_logger(), "Initial pose rejected: invalid covariance");
        return;
      }
    map_base.linear() = rotation.toRotationMatrix();
    pending_hint_ = m;
    manual_pose_ = map_base;
    preview_base_ =
        manual_preview_enabled_ ? std::optional<Pose>(map_base) : std::nullopt;
    manual_active_ = true;
    manual_failed_ = false;
    manual_started_ = std::chrono::steady_clock::now();
    manual_phase_ = "WAITING TF";
    localizer_->manual(std::nullopt, std::nullopt, true);
    tf_trusted_ = false;
    clearPendingNavigation();
    publishStatus();
    cancelNavigation();
    // 仅预览允许使用已完成源帧的底盘变换；实际几何初值仍查询点击的原始时刻。
    if (preview_base_ && cached_odom_base_)
      publishPreview(*cached_odom_base_, last_tf_stamp_ + 1);
    publishRecoveryMarkers();
    clearNavigationMaps();
    processPendingHint();
  }
  /// 同刻 TF 到齐后安装提示并立即启动快速配准，避免重复清空扫描或重建代次。
  void processPendingHint() {
    if (!pending_hint_ || !latest_frame_ || !manual_active_)
      return;
    const auto &m = *pending_hint_;
    const auto stamp =
        rclcpp::Time(m.header.stamp, get_clock()->get_clock_type());
    try {
      const Pose world_base(tf2::transformToEigen(tf_buffer_->lookupTransform(
          sensor_world_frame_, base_frame_, stamp)));
      const auto world_imu = frontendPoseAt(stamp.nanoseconds());
      if (!world_imu)
        return;
      HintLimits limits;
      const double variance =
          std::max(m.pose.covariance[0], m.pose.covariance[7]);
      limits.translation = variance > 0 ? std::clamp(3. * std::sqrt(variance),
                                                     .5, hint_max_translation_)
                                        : hint_max_translation_;
      limits.rotation = m.pose.covariance[35] > 0
                            ? std::clamp(3. * std::sqrt(m.pose.covariance[35]),
                                         .1745329252, hint_max_rotation_)
                            : hint_max_rotation_;
      localizer_->updateHint(*manual_pose_ * world_base.inverse() * *world_imu,
                             *world_imu, limits);
      pending_hint_.reset();
      manual_phase_ = "VERIFYING / GLOBAL FALLBACK";
      localizer_->requestRecovery(*latest_frame_);
      RCLCPP_INFO(get_logger(),
                  "Initial pose accepted for verification; translation "
                  "bound=%.2f m rotation bound=%.1f deg",
                  limits.translation, limits.rotation * 180. / M_PI);
    } catch (const tf2::TransformException &e) {
      RCLCPP_DEBUG(get_logger(), "Initial pose waiting for source-time TF: %s",
                   e.what());
    }
  }
  /// 完成 UI 请求不等于直接授予运动许可；最终状态仍检查已验证图锚和输出 TF。
  void completeManual() {
    manual_active_ = false;
    preview_base_.reset();
    manual_failed_ = false;
    manual_elapsed_ = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - manual_started_)
                          .count();
    manual_phase_ = "LOCALIZED";
    clearNavigationMaps();
    clearRecoveryMarkers();
  }
  /// 预览只改变导航显示，不更新图锚或最后可信位姿，也不授予运动许可。
  void publishPreview(const Pose &odom_base, std::int64_t stamp) {
    if (!preview_base_ || !publish_tf_ || stamp <= 0 ||
        (tf_current_ && stamp <= last_tf_stamp_))
      return;
    auto transform =
        tf2::eigenToTransform(Pose(*preview_base_ * odom_base.inverse()));
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.header.stamp = rclcpp::Time(stamp, get_clock()->get_clock_type());
    tf_broadcaster_->sendTransform(transform);
    tf_current_ = true;
    tf_trusted_ = false;
    last_tf_stamp_ = stamp;
  }
  /// 仅对当前身份发起一次重置；服务受理不代表完成，新 epoch 数据才解除等待。
  bool requestReset(const char *reason = "manual reset service") {
    if (!localizer_ || !latest_frame_ || !reset_client_->service_is_ready() ||
        reset_pending_ ||
        (reset_attempted_ && *reset_attempted_ == latest_frame_->identity))
      return false;
    using Service = rm27_localization_interfaces::srv::ResetFrontend;
    auto request = std::make_shared<Service::Request>();
    request->instance_id = latest_frame_->identity.instance;
    request->epoch = latest_frame_->identity.epoch;
    request->request_id =
        std::chrono::system_clock::now().time_since_epoch().count();
    reset_attempted_ = latest_frame_->identity;
    reset_pending_ = true;
    reset_started_ = std::chrono::steady_clock::now();
    reset_expected_epoch_ = request->epoch + 1;
    RCLCPP_WARN(get_logger(),
                "Frontend reset requested: reason=%s epoch=%lu sequence=%lu",
                reason, static_cast<unsigned long>(request->epoch),
                static_cast<unsigned long>(latest_frame_->sequence));
    localizer_->resetSegment();
    clearPendingNavigation();
    reset_client_->async_send_request(
        request, [this, identity = *reset_attempted_](
                     rclcpp::Client<Service>::SharedFuture future) {
          if (!reset_pending_ || !reset_attempted_ ||
              !(*reset_attempted_ == identity))
            return;
          const auto response = future.get();
          if (!response->accepted) {
            reset_pending_ = false;
            RCLCPP_WARN(get_logger(), "Frontend reset rejected: %s",
                        response->reason.c_str());
          }
        });
    return true;
  }
  /// 独立墙钟维护确保前端断流仍能及时撤销可信输出，不伪造新的传感器时间。
  void maintain() {
    if (localizer_ && localizer_->ready() && !background_published_) {
      const auto cloud = localizer_->background();
      if (cloud) {
        sensor_msgs::msg::PointCloud2 message;
        message.header.frame_id = map_frame_;
        message.header.stamp = now();
        sensor_msgs::PointCloud2Modifier modifier(message);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud->size());
        sensor_msgs::PointCloud2Iterator<float> x(message, "x"),
            y(message, "y"), z(message, "z");
        for (const auto &p : *cloud) {
          *x = p.x;
          *y = p.y;
          *z = p.z;
          ++x;
          ++y;
          ++z;
        }
        background_pub_->publish(message);
        background_published_ = true;
      }
    }
    processPendingHint();
    if (manual_active_) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        manual_started_)
              .count();
      if (elapsed > manual_timeout_ ||
          (pending_hint_ && elapsed > manual_tf_timeout_)) {
        manual_phase_ =
            pending_hint_
                ? "FAILED: TF TIMEOUT"
                : (localizer_->reason().find("ambiguous") != std::string::npos
                       ? "FAILED: AMBIGUOUS LOCATIONS"
                       : "FAILED: RECOVERY DEADLINE");
        localizer_->cancelRecovery(manual_phase_);
        pending_hint_.reset();
        preview_base_.reset();
        manual_active_ = false;
        manual_failed_ = true;
        manual_elapsed_ = elapsed;
        clearNavigationMaps();
      } else if (!pending_hint_ &&
                 localizer_->reason().find("ambiguous") != std::string::npos) {
        manual_phase_ = "AMBIGUOUS: WAITING FOR NEW EVIDENCE";
      }
    }
    if (reset_pending_ && std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - reset_started_)
                                  .count() > reset_timeout_) {
      reset_pending_ = false;
      RCLCPP_ERROR(get_logger(),
                   "Frontend reset completion timed out; remaining untrusted");
    }
    if (localizer_ && last_receive_ &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      *last_receive_)
                .count() > input_timeout_) {
      const auto wall_now = std::chrono::steady_clock::now();
      const double admitted_age =
          std::chrono::duration<double>(wall_now - *last_receive_).count();
      const double callback_age = last_raw_receive_
          ? std::chrono::duration<double>(wall_now - *last_raw_receive_).count()
          : -1.0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Frontend admitted-frame timeout: admitted_wall_age=%.3f "
          "callback_wall_age=%.3f threshold=%.3f last_reason=%s",
          admitted_age, callback_age, input_timeout_, last_reason_.c_str());
      localizer_->inputTimedOut();
      if (auto_reset_)
        requestReset("admitted-frame timeout");
    }
    if (localizer_ && (!localizer_->state().trusted() || reset_pending_)) {
      clearPendingNavigation();
      if (latest_frame_ && (last_output_pose_ || preview_base_))
        publishNavigation(*latest_frame_,
                          last_output_pose_.value_or(Pose::Identity()));
    } else {
      flushNavigationOutputs();
    }
    publishRecoveryMarkers();
    publishStatus();
  }
  /// 队列身份同时约束前端坐标、图段和定位状态机代次。
  NavigationOutputQueue<nav_msgs::msg::Odometry>::Context navigationContext() const {
    return {latest_frame_->identity, manager_->segment(),
            localizer_ ? localizer_->state().generation() : 0};
  }
  void clearPendingNavigation() {
    pending_navigation_.clear();
    tf_trusted_ = false;
    if (navigation_retry_)
      navigation_retry_->cancel();
  }
  /// 发布的是同刻估计，后到的最新原子帧不要求把这条输出冒充为最新时刻。
  bool navigationOutputTrusted() const {
    if (reset_pending_ || manual_active_ || !latest_frame_ || !localizer_ ||
        !localizer_->anchored() || !localizer_->state().trusted() ||
        !last_output_context_ || !last_output_frame_ ||
        !(*last_output_context_ == navigationContext()) ||
        !pending_navigation_.fresh(last_output_received_, last_output_stamp_,
                                   std::chrono::steady_clock::now(),
                                   latest_frame_->stamp_ns))
      return false;
    const double source_age =
        (now() - rclcpp::Time(last_output_stamp_, get_clock()->get_clock_type()))
            .seconds();
    if (source_age < 0 || source_age > .5)
      return false;
    return !publish_tf_ ||
           (tf_current_ && tf_trusted_ && last_output_stamp_ == last_tf_stamp_);
  }
  /// TF 和后端里程计完成后一起提交水位；对齐消息与恢复参考也沿用该帧时间。
  void publishEstimate(const Frame &f, const Pose &estimated,
                       const nav_msgs::msg::Odometry &out,
                       std::chrono::steady_clock::time_point received) {
    estimate_pub_->publish(out);
    last_output_pose_ = estimated;
    last_output_stamp_ = f.stamp_ns;
    last_output_frame_ = f;
    last_output_received_ = received;
    last_output_context_ = navigationContext();
    if (!publish_tf_ && manual_active_ && localizer_ &&
        localizer_->state().trusted())
      completeManual();
    if (mapMode() && localizer_->anchored()) {
      localizer_->rememberPose(estimated, f.stamp_ns);
      geometry_msgs::msg::TransformStamped alignment;
      alignment.header.stamp = out.header.stamp;
      alignment.header.frame_id = map_frame_;
      alignment.child_frame_id = sensor_world_frame_;
      alignment.transform = tf2::eigenToTransform(
          Eigen::Isometry3d(estimated * f.odom.inverse())).transform;
      alignment_pub_->publish(alignment);
      last_reason_ = f.degraded ? "degraded odometry; map anchored"
                                : localizer_->reason();
    } else {
      last_reason_ = f.degraded ? "degraded odometry"
                                : "odometry only; globally unanchored";
    }
  }
  void flushNavigationOutputs() {
    if (!publish_tf_ || !latest_frame_ || !localizer_ || reset_pending_ ||
        !localizer_->state().trusted()) {
      clearPendingNavigation();
      return;
    }
    pending_navigation_.synchronize(navigationContext());
    pending_navigation_.publishNewestReady(
        std::chrono::steady_clock::now(), latest_frame_->stamp_ns,
        last_tf_stamp_, [this](const auto &sample) {
          if (sample.frame.stamp_ns > now().nanoseconds())
            return false;
          if (!publishNavigation(sample.frame, sample.estimate))
            return false;
          publishEstimate(sample.frame, sample.estimate, sample.payload,
                          sample.received);
          return true;
        });
    if (!pending_navigation_.size())
      navigation_retry_->cancel();
  }
  /// 发布结构化可信状态；导航还需检查 heartbeat 与 pose_stamp 的新鲜度。
  void publishStatus() {
    // 自动恢复也须清理先前失败的 UI；不能只在人工请求完成分支删除。
    if (localizer_ && localizer_->state().trusted() && !manual_active_)
      clearRecoveryMarkers();
    rm27_localization_interfaces::msg::LocalizationStatus m;
    m.header.stamp = now();
    m.header.frame_id = map_frame_;
    m.pose_stamp = last_stamp_;
    if (publish_tf_ && tf_current_ && last_tf_stamp_ > 0)
      m.pose_stamp =
          rclcpp::Time(last_tf_stamp_, get_clock()->get_clock_type());
    m.state = localizer_ ? localizer_->state().name()
                         : (manager_->latest() ? "NORMAL" : "INITIALIZING");
    m.reason = localizer_ ? localizer_->reason() : last_reason_;
    m.generation =
        localizer_ ? localizer_->state().generation() : manager_->segment();
    m.segment = manager_->segment();
    m.map_ready = localizer_ && localizer_->ready();
    m.global_trusted = navigationOutputTrusted();
    m.tf_published = publish_tf_ && tf_current_ && last_tf_stamp_ > 0;
    m.frozen = localizer_ && !localizer_->state().trusted() &&
               (last_trusted_base_.has_value() || preview_base_.has_value());
    if (latest_frame_) {
      m.instance_id = latest_frame_->identity.instance;
      m.epoch = latest_frame_->identity.epoch;
    }
    if (m.state != last_logged_state_ ||
        (localizer_ && localizer_->failed() && !map_failure_logged_)) {
      RCLCPP_INFO(get_logger(),
                  "Localization state=%s map_ready=%d trusted=%d reason=%s",
                  m.state.c_str(), m.map_ready, m.global_trusted,
                  m.reason.c_str());
      last_logged_state_ = m.state;
      map_failure_logged_ = localizer_ && localizer_->failed();
    }
    if (!m.global_trusted && m.map_ready)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Waiting for verified map localization: %s",
                           m.reason.c_str());
    status_pub_->publish(m);
  }
  /// 用同刻 world←odom 与 IMU 位姿建立导航
  /// TF；恢复冻结采用底盘参考点而非活动雷达。
  bool publishNavigation(const Frame &f, const Pose &estimated) {
    // 尚未到齐的新帧不作废同代已发布的一致输出；可信状态另验原始时间和年龄。
    // 不可信阶段仍可发布冻结 TF，但不能继承正常输出许可。
    if (!localizer_ || !localizer_->state().trusted())
      tf_trusted_ = false;
    if (!publish_tf_ || !localizer_ ||
        (!localizer_->anchored() && !last_trusted_base_ && !preview_base_))
      return !publish_tf_;
    if (f.stamp_ns < last_tf_stamp_)
      return false;
    try {
      const auto stamp =
          rclcpp::Time(f.stamp_ns, get_clock()->get_clock_type());
      const Pose world_odom(tf2::transformToEigen(tf_buffer_->lookupTransform(
          sensor_world_frame_, odom_frame_, stamp)));
      const Pose odom_base(tf2::transformToEigen(
          tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp)));
      cached_odom_base_ = odom_base;
      if (preview_base_ && !localizer_->state().trusted()) {
        publishPreview(odom_base, f.stamp_ns);
        return tf_current_;
      }
      // 预览使用更晚的独立时间点，不覆盖已提交的历史 TF。
      // 新的可信输出必须在它之后，避免旧时间的结果被预览挡在 TF 缓存中。
      if (manual_active_ && localizer_->state().trusted() &&
          f.stamp_ns <= last_tf_stamp_)
        return false;
      if (manual_active_ && localizer_->state().trusted())
        completeManual();
      Pose map_odom = estimated * f.odom.inverse() * world_odom;
      if (localizer_->state().trusted()) {
        last_trusted_base_ = map_odom * odom_base;
        if (manual_pose_ && !manual_active_ && !manual_failed_)
          manual_pose_ = last_trusted_base_;
      } else if (last_trusted_base_)
        map_odom = *last_trusted_base_ * odom_base.inverse();
      else
        return false;
      auto transform = tf2::eigenToTransform(map_odom);
      transform.header.stamp = stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = odom_frame_;
      tf_broadcaster_->sendTransform(transform);
      tf_current_ = true;
      tf_trusted_ = localizer_->state().trusted() && !manual_active_;
      last_tf_stamp_ = f.stamp_ns;
      return true;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Navigation TF pending at %.6f: %s",
                           f.stamp_ns * 1e-9, e.what());
      return false;
    }
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
    last_raw_receive_ = start;
    ++received_;
    try {
      Frame f = adapter_->decode(m);
      const auto admission = manager_->observe(f);
      if (admission == Admission::Rejected) {
        ++rejected_;
        last_reason_ = manager_->reason();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Rejected backend frame admission: %s",
                             last_reason_.c_str());
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
      if (reset_pending_ && reset_attempted_ &&
          (f.identity.instance != reset_attempted_->instance ||
           f.identity.epoch >= reset_expected_epoch_))
        reset_pending_ = false;
      if (!odom_history_.empty() &&
          odom_history_.back().identity != f.identity) {
        odom_history_.clear();
        pending_hint_.reset();
        preview_base_.reset();
        manual_active_ = false;
        cached_odom_base_.reset();
        tf_current_ = tf_trusted_ = false;
        last_tf_stamp_ = -1;
        last_output_frame_.reset();
        clearPendingNavigation();
      }
      odom_history_.push_back(f);
      while (odom_history_.size() > 100)
        odom_history_.pop_front();
      latest_frame_ = f;
      last_receive_ = std::chrono::steady_clock::now();
      Cloud::ConstPtr local_cloud;
      if (localizer_) {
        local_cloud = adapter_->cloud(m, f);
        localizer_->observe(f, local_cloud);
        if (!startup_hint_sent_ && startup_pose_enabled_ &&
            f.identity.epoch == 0) {
          try {
            // 出生点给的是底盘参考点，需用同刻世界←底盘与原子 IMU 位姿换算。
            // 不查询尚未建立的 map TF，也不假设 body 是可查询的 TF 节点。
            const Pose world_base(
                tf2::transformToEigen(tf_buffer_->lookupTransform(
                    sensor_world_frame_, base_frame_,
                    rclcpp::Time(f.stamp_ns, get_clock()->get_clock_type()))));
            localizer_->manual(
                startup_map_base_ * world_base.inverse() * f.odom, f.odom);
            startup_hint_sent_ = true;
            localizer_->observe(f, local_cloud);
            RCLCPP_INFO(get_logger(), "Configured startup base pose submitted "
                                      "for geometric verification");
          } catch (const tf2::TransformException &e) {
            RCLCPP_DEBUG(get_logger(),
                         "Waiting for startup source-time base TF: %s",
                         e.what());
          }
        }
        if (!startup_hint_sent_ && seed_source_ == "parameter") {
          localizer_->manual(initial_alignment_ * f.odom, f.odom);
          startup_hint_sent_ = true;
          localizer_->observe(f, local_cloud);
        }
      }
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
          last_covariance_ =
              manager_->graph().covariance(manager_->latest()->id);
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
                      "rel=%.5f,%.5f,%.5f,%.7f,%.7f,%.7f,%.7f nis=%.3f "
                      "rmse=%.4f overlap=%.3f",
                      r.measurement.measurement_id.c_str(),
                      long(r.from_stamp_ns), long(r.to_stamp_ns), t.x(), t.y(),
                      t.z(), q.x(), q.y(), q.z(), q.w(), r.nis, r.rmse,
                      r.overlap);
        }
        if (outcome.graph_failure) {
          manager_->breakSegment();
          loop_->syncSegment(manager_->segment());
          ++graph_failures_;
        } else if (outcome.accepted && manager_->latest()) {
          last_covariance_ =
              manager_->graph().covariance(manager_->latest()->id);
        }
      }
      if (reset_pending_) {
        publishStatus();
        finishTiming(start);
        return;
      }
      // 不可信阶段仍接收前端源帧，但不跨冻结区间建立普通里程计边。
      std::optional<PosePrior> recovery_prior;
      if (localizer_ && !localizer_->state().trusted()) {
        recovery_prior = localizer_->takeAnchor(f);
        if (!recovery_prior) {
          localizer_->requestRecovery(f);
          if (last_output_pose_ || preview_base_)
            publishNavigation(f, last_output_pose_.value_or(Pose::Identity()));
          publishStatus();
          finishTiming(start);
          return;
        }
      }
      if (manager_->shouldCreate(f)) {
        auto cloud = local_cloud ? local_cloud : adapter_->cloud(m, f);
        const auto prior = recovery_prior;
        const auto graph_start = std::chrono::steady_clock::now();
        Keyframe k;
        try {
          k = manager_->commit(f, prior);
          if (prior && localizer_)
            localizer_->confirmAnchor();
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
        if (mapMode() && localizer_->anchored() &&
            manager_->lastMeasurement()) {
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
        if (publish_tf_) {
          pending_navigation_.synchronize(navigationContext());
          pending_navigation_.push(f, estimated, out, *last_receive_);
          flushNavigationOutputs();
          if (pending_navigation_.size() && navigation_retry_->is_canceled())
            navigation_retry_->reset();
        } else {
          publishEstimate(f, estimated, out, *last_receive_);
        }
        publishStatus();
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
    put("navigation_tf_pending", pending_navigation_.size());
    put("navigation_tf_expired", pending_navigation_.expired());
    put("navigation_tf_overflow", pending_navigation_.overflow());
    put("navigation_output_stamp_ns", last_output_stamp_);
    put("navigation_output_wall_age", last_output_frame_
            ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           last_output_received_).count()
            : -1.);
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
      put("loop_last_edge",
          std::to_string(l.last_from) + "->" + std::to_string(l.last_to));
      put("loop_verify_ms", l.last_ms);
      put("loop_verify_ms_max", l.max_ms);
      put("loop_verify_ms_total", l.total_ms);
      put("loop_nis_ms_max", l.nis_ms_max);
      put("loop_update_ms_max", l.update_ms_max);
    }
    if (localizer_) {
      const auto m = localizer_->stats();
      if (localizer_->failed())
        s.level = 2;
      put("map_state", localizer_->state().name());
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
      put("map_consecutive_rmse_failures", m.consecutive_rmse_failures);
      put("map_last_rmse", m.last_rmse);
      put("map_last_inlier_ratio", m.last_inlier_ratio);
      put("map_last_rank", m.last_rank);
      put("map_last_nis", m.last_nis);
      put("map_last_correction_m", m.last_correction);
      put("motion_recovery_active", m.motion_recovery_active);
      put("motion_recovery_radius_m", m.motion_recovery_radius);
      put("motion_recovery_angle_deg", m.motion_recovery_angle * 180. / M_PI);
      put("motion_recovery_reason", m.motion_recovery_reason);
      put("map_registration_attempts", m.last_attempts);
      put("map_cancelled_jobs", m.cancelled);
      put("map_registration_ms", m.last_ms);
      put("map_registration_ms_max", m.max_ms);
      put("map_registration_ms_total", m.total_ms);
    }
    put("publishes_tf", publish_tf_);
    out.status.push_back(s);
    diagnostic_pub_->publish(out);
  }
  bool recovery_markers_visible_ = true; // 启动时发送删除消息清理旧 RViz 残留。
  bool manual_preview_enabled_ = true, manual_active_ = false,
       manual_failed_ = false, tf_trusted_ = false;
  double manual_timeout_ = 5., manual_tf_timeout_ = 1., manual_elapsed_ = 0.;
  double hint_max_translation_ = 2., hint_max_rotation_ = 1.0471975512;
  std::chrono::steady_clock::time_point manual_started_;
  std::optional<Pose> manual_pose_, preview_base_, cached_odom_base_;
  std::string manual_phase_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      recovery_markers_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_navigation_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_,
      clear_local_;
  bool auto_reset_ = true, reset_pending_ = false;
  double reset_timeout_ = 5.;
  std::uint64_t reset_expected_epoch_ = 0;
  std::optional<Identity> reset_attempted_;
  std::chrono::steady_clock::time_point reset_started_;
  rclcpp::Client<rm27_localization_interfaces::srv::ResetFrontend>::SharedPtr
      reset_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::string last_logged_state_;
  bool map_failure_logged_ = false;
  std::string bundle_, odom_frame_, base_frame_;
  bool startup_pose_enabled_ = false;
  Pose startup_map_base_ = Pose::Identity();
  bool publish_tf_ = false, tf_current_ = false, startup_hint_sent_ = false,
       background_published_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr background_pub_;
  double input_timeout_ = 1.;
  std::int64_t last_output_stamp_ = 0, last_tf_stamp_ = -1;
  std::optional<Frame> latest_frame_;
  std::deque<Frame> odom_history_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pending_hint_;
  std::optional<Pose> last_trusted_base_, last_output_pose_;
  using NavigationQueue = NavigationOutputQueue<nav_msgs::msg::Odometry>;
  NavigationQueue pending_navigation_;
  std::optional<NavigationQueue::Context> last_output_context_;
  std::optional<Frame> last_output_frame_;
  std::chrono::steady_clock::time_point last_output_received_;
  rclcpp::TimerBase::SharedPtr navigation_retry_;
  std::optional<std::chrono::steady_clock::time_point> last_receive_;
  std::optional<std::chrono::steady_clock::time_point> last_raw_receive_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<rm27_localization_interfaces::msg::LocalizationStatus>::
      SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service_;
  rclcpp::TimerBase::SharedPtr maintenance_;
  std::string map_frame_, seed_source_, sensor_world_frame_;
  Pose initial_alignment_ = Pose::Identity();
  int source_keyframes_ = 3;
  double source_max_distance_ = 1.5;
  std::unique_ptr<LocalizationController> localizer_;
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
