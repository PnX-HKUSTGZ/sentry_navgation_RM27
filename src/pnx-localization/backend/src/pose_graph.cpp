#include "rm27_backend/pose_graph.hpp"
#include <Eigen/SVD>
#include <boost/math/distributions/chi_squared.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace rm27_backend {
namespace {
/// 将 Eigen 刚体位姿转换为 GTSAM Pose3。
gtsam::Pose3 toGtsam(const Pose &p) { return gtsam::Pose3(p.matrix()); }
/// 将 GTSAM Pose3 转换为后端刚体位姿。
Pose fromGtsam(const gtsam::Pose3 &p) { return Pose(p.matrix()); }
/// 在后端 [平移, 旋转] 与 GTSAM [旋转, 平移] 顺序之间交换协方差块。
/// 同时交换交叉项；该操作自身就是逆变换。
Covariance reorder(const Covariance &p) {
  Covariance r;
  r.topLeftCorner<3, 3>() = p.bottomRightCorner<3, 3>();
  r.topRightCorner<3, 3>() = p.bottomLeftCorner<3, 3>();
  r.bottomLeftCorner<3, 3>() = p.topRightCorner<3, 3>();
  r.bottomRightCorner<3, 3>() = p.topLeftCorner<3, 3>();
  return r;
}
/// 验证 1～6 行平方根信息的有限性和行满秩，再转换到 GTSAM 分量顺序。
Eigen::MatrixXd information(const Eigen::MatrixXd &l) {
  if (l.cols() != 6 || l.rows() < 1 || l.rows() > 6 || !l.allFinite())
    throw std::invalid_argument("invalid square-root information");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(l);
  if (svd.rank() != l.rows())
    throw std::invalid_argument("information rows are rank deficient");
  Eigen::MatrixXd r(l.rows(), 6);
  r.leftCols(3) = l.rightCols(3);
  r.rightCols(3) = l.leftCols(3);
  return r;
}
/// 按平方根信息投影绝对位姿残差的因子，允许只约束部分可观方向。
class AbsoluteFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  gtsam::Pose3 z_;
  Eigen::MatrixXd l_;

public:
  /// 保存绝对位姿测量与 GTSAM 顺序的信息矩阵，投影后残差使用单位噪声。
  AbsoluteFactor(Key k, const Pose &z, const Eigen::MatrixXd &l)
      : NoiseModelFactor1(gtsam::noiseModel::Unit::Create(l.rows()), k),
        z_(toGtsam(z)), l_(l) {}
  /// 计算测量与当前位姿的局部坐标残差，并在需要时输出投影后的解析雅可比。
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &x,
      boost::optional<gtsam::Matrix &> h = boost::none) const override {
    gtsam::Matrix66 j;
    const auto r = z_.localCoordinates(x, boost::none, j);
    if (h)
      *h = l_ * j;
    return l_ * r;
  }
};
/// 连接两个活动位姿的回环因子，支持部分秩信息并使用 Cauchy 鲁棒核。
/// 鲁棒核降低异常残差影响，不能替代提交前的几何和回环一致性验收。
class LoopFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  gtsam::Pose3 z_;
  Eigen::MatrixXd l_;

public:
  /// 保存 from←to 测量及信息矩阵，按残差维数设置鲁棒核尺度。
  LoopFactor(Key a, Key b, const Pose &z, const Eigen::MatrixXd &l, double size)
      : NoiseModelFactor2(gtsam::noiseModel::Robust::Create(
                              gtsam::noiseModel::mEstimator::Cauchy::Create(
                                  size * std::sqrt(double(l.rows()))),
                              gtsam::noiseModel::Unit::Create(l.rows())),
                          a, b),
        z_(toGtsam(z)), l_(l) {}
  /// 比较预测相对位姿与回环测量，按链式法则计算两个端点的投影雅可比。
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &a, const gtsam::Pose3 &b,
      boost::optional<gtsam::Matrix &> ha = boost::none,
      boost::optional<gtsam::Matrix &> hb = boost::none) const override {
    gtsam::Matrix66 ja, jb, j;
    const auto p = a.between(b, ja, jb);
    const auto r = z_.localCoordinates(p, boost::none, j);
    if (ha)
      *ha = l_ * j * ja;
    if (hb)
      *hb = l_ * j * jb;
    return l_ * r;
  }
};
} // 匿名命名空间
/// 检查窗口、先验和线性化参数，再初始化第零轨迹段。
PoseGraph::PoseGraph(GraphConfig c) : config_(c) {
  if (!std::isfinite(c.lag_seconds) || c.lag_seconds < 0 ||
      (c.max_states && c.max_states < 3) ||
      !std::isfinite(c.prior_translation_std) || c.prior_translation_std <= 0 ||
      !std::isfinite(c.prior_rotation_std) || c.prior_rotation_std <= 0 ||
      !std::isfinite(c.relinearize_threshold) || c.relinearize_threshold <= 0 ||
      !std::isfinite(c.loop_cauchy_size) || c.loop_cauchy_size <= 0)
    throw std::invalid_argument("invalid graph configuration");
  reset(0);
}
/// 重新创建 iSAM2，清空活动时间表和观测身份，隔离不同轨迹段。
void PoseGraph::reset(std::uint64_t segment) {
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = config_.relinearize_threshold;
  p.relinearizeSkip = 1;
  p.findUnusedFactorSlots = true;
  isam_ = std::make_unique<gtsam::ISAM2>(p);
  times_.clear();
  observations_.clear();
  segment_ = segment;
  marginalized_ = 0;
}
/// 查询求解器是否仍持有指定节点。
bool PoseGraph::contains(Key k) const { return isam_->valueExists(k); }
/// 执行带新因子的增量更新及一次后续更新。
/// 异常可能使求解器处于不完整状态，调用方必须通过重置或断段恢复。
void PoseGraph::submit(const gtsam::NonlinearFactorGraph &g,
                       const gtsam::Values &v) {
  // 更新异常会使估计器状态不再可信，调用方必须开启新的轨迹段。
  isam_->update(g, v);
  isam_->update();
}
/// 以原始首帧位姿和配置标准差构造规范先验，固定本段坐标自由度。
/// 先验只定义局部坐标基准，不提供独立的地图定位信息。
void PoseGraph::addFirst(const Keyframe &k) {
  PosePrior prior;
  prior.pose = k.frame.odom;
  prior.covariance.setZero();
  prior.covariance.diagonal().head<3>().setConstant(
      std::pow(config_.prior_translation_std, 2));
  prior.covariance.diagonal().tail<3>().setConstant(
      std::pow(config_.prior_rotation_std, 2));
  addFirst(k, prior);
}
/// 校验先验位姿与协方差后插入首个节点；先验的坐标系即本段图坐标系。
void PoseGraph::addFirst(const Keyframe &k, const PosePrior &prior) {
  if (!times_.empty() || k.segment != segment_ || !validPose(prior.pose) ||
      !positiveCovariance(prior.covariance))
    throw std::invalid_argument("invalid first graph state");
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  g.add(gtsam::PriorFactor<gtsam::Pose3>(
      k.id, toGtsam(prior.pose),
      gtsam::noiseModel::Gaussian::Covariance(reorder(prior.covariance))));
  v.insert(k.id, toGtsam(prior.pose));
  submit(g, v);
  times_.emplace_back(k.id, k.frame.stamp_ns);
}
/// 校验前后节点、时序和测量协方差，添加 BetweenFactor 并登记新状态。
/// 初值由已估计的前驱位姿与原始里程计增量组合，提交后执行窗口裁剪。
void PoseGraph::addOdometry(const Keyframe &k, const OdometryMeasurement &m) {
  if (times_.empty() || k.segment != segment_ ||
      m.from != times_.back().first || m.to != k.id || contains(k.id) ||
      k.frame.stamp_ns <= times_.back().second || !validPose(m.relative) ||
      !positiveCovariance(m.covariance))
    throw std::invalid_argument("invalid odometry factor");
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  g.add(gtsam::BetweenFactor<gtsam::Pose3>(
      m.from, m.to, toGtsam(m.relative),
      gtsam::noiseModel::Gaussian::Covariance(reorder(m.covariance))));
  v.insert(k.id, toGtsam(estimate(m.from) * m.relative));
  submit(g, v);
  times_.emplace_back(k.id, k.frame.stamp_ns);
  trim();
}
/// 根据时间和节点数量选择过期前缀，重排后边缘化以保留联合线性信息。
/// 因子空槽过多时保留全部有效因子重建求解器，避免长期增长；同步清理过期观测身份。
void PoseGraph::trim() {
  std::size_t count = 0;
  while (
      count + 1 < times_.size() &&
      ((config_.lag_seconds > 0 &&
        (times_.back().second - times_[count].second) * 1e-9 >
            config_.lag_seconds) ||
       (config_.max_states > 0 && times_.size() - count > config_.max_states)))
    ++count;
  if (!count)
    return;
  gtsam::KeyList expired, re_eliminate;
  gtsam::FastMap<gtsam::Key, int> groups;
  for (std::size_t i = 0; i < times_.size(); ++i) {
    const Key key = times_[i].first;
    groups[key] = i < count ? 0 : 1;
    re_eliminate.push_back(key);
    if (i < count)
      expired.push_back(key);
  }
  // 边缘化前重排有界活动图，使旧位姿处于可消元的叶端或前部变量位置；
  // 后续回环因子连接历史团时，也必须满足这一条件。
  isam_->update(gtsam::NonlinearFactorGraph{}, gtsam::Values{}, {}, groups,
                boost::none, re_eliminate);
  isam_->marginalizeLeaves(expired);
  for (std::size_t i = 0; i < count; ++i)
    times_.pop_front();
  marginalized_ += count;
  // GTSAM 4.2 边缘化后可能留下未复用的空因子槽。
  // 重建时保留全部有效因子，包括联合线性边缘信息。
  if (isam_->getFactorsUnsafe().size() > 2 * times_.size() + 16) {
    gtsam::NonlinearFactorGraph live;
    for (const auto &factor : isam_->getFactorsUnsafe())
      if (factor)
        live.push_back(factor);
    const gtsam::Values values = isam_->calculateEstimate();
    gtsam::ISAM2Params p;
    p.relinearizeThreshold = config_.relinearize_threshold;
    p.relinearizeSkip = 1;
    p.findUnusedFactorSlots = true;
    auto compact = std::make_unique<gtsam::ISAM2>(p);
    compact->update(live, values);
    compact->update();
    isam_ = std::move(compact);
  }
  for (auto it = observations_.begin(); it != observations_.end();) {
    if (!contains(it->second.first) || !contains(it->second.second))
      it = observations_.erase(it);
    else
      ++it;
  }
}
/// 接收已验收的地图位姿测量并立即更新图。
/// 段不匹配、重复 ID、容量超限或节点过期返回 false；非法位姿/信息矩阵抛异常。
bool PoseGraph::addAbsolute(const AbsolutePoseMeasurement &m) {
  if (m.segment != segment_ || m.measurement_id.empty() ||
      m.measurement_id.size() > 256 || observations_.size() >= 1024 ||
      observations_.count(m.measurement_id) || !contains(m.key))
    return false;
  if (!validPose(m.pose))
    throw std::invalid_argument("invalid map pose");
  gtsam::NonlinearFactorGraph g;
  g.emplace_shared<AbsoluteFactor>(m.key, m.pose,
                                   information(m.sqrt_information));
  submit(g, gtsam::Values{});
  observations_[m.measurement_id] = {m.key, m.key};
  return true;
}
/// 白化新息 e = L·Local(z, x̂)，雅可比 J = L·∂Local/∂x，协方差 S = I + J P Jᵀ。
/// 预测协方差 P 取 iSAM2 当前边缘协方差；被拒观测不会进入图，避免错误匹配污染估计。
Innovation PoseGraph::assessAbsolute(const AbsolutePoseMeasurement &m) const {
  Innovation out;
  if (m.segment != segment_ || !contains(m.key) || !validPose(m.pose))
    return out;
  Eigen::MatrixXd l;
  try {
    l = information(m.sqrt_information);
  } catch (const std::exception &) {
    return out;
  }
  const gtsam::Pose3 x = isam_->calculateEstimate<gtsam::Pose3>(m.key);
  gtsam::Matrix66 j;
  const gtsam::Vector6 r = toGtsam(m.pose).localCoordinates(x, boost::none, j);
  const Eigen::VectorXd e = l * r;
  const Eigen::MatrixXd h = l * j;
  Eigen::MatrixXd s = Eigen::MatrixXd::Identity(l.rows(), l.rows()) +
                      h * isam_->marginalCovariance(m.key) * h.transpose();
  s = .5 * (s + s.transpose()).eval();
  Eigen::LDLT<Eigen::MatrixXd> ldlt(s);
  if (ldlt.info() != Eigen::Success || (ldlt.vectorD().array() <= 1e-12).any())
    return out;
  out.nis = std::max(0., e.dot(ldlt.solve(e)));
  out.dof = int(l.rows());
  try {
    out.probability = boost::math::cdf(boost::math::complement(
        boost::math::chi_squared(double(out.dof)), out.nis));
  } catch (const std::exception &) {
    return out;
  }
  out.valid = std::isfinite(out.nis) && std::isfinite(out.probability);
  return out;
}
/// 回环新息 e = L·Local(z, x̂_a⁻¹x̂_b)，H = [L·J·J_a, L·J·J_b]，S = I + H P_ab Hᵀ。
/// P_ab 为两端联合边缘协方差，由当前图全量线性化得到；仅在回环提交前调用，开销可接受。
Innovation PoseGraph::assessLoop(const LoopMeasurement &m) const {
  Innovation out;
  if (m.segment != segment_ || m.from == m.to || !contains(m.from) ||
      !contains(m.to) || !validPose(m.relative))
    return out;
  Eigen::MatrixXd l;
  try {
    l = information(m.sqrt_information);
  } catch (const std::exception &) {
    return out;
  }
  const gtsam::Values values = isam_->calculateEstimate();
  const auto a = values.at<gtsam::Pose3>(m.from);
  const auto b = values.at<gtsam::Pose3>(m.to);
  gtsam::Matrix66 ja, jb, j;
  const gtsam::Pose3 predicted = a.between(b, ja, jb);
  const gtsam::Vector6 r = toGtsam(m.relative).localCoordinates(predicted, boost::none, j);
  const Eigen::VectorXd e = l * r;
  Eigen::MatrixXd h(l.rows(), 12);
  h.leftCols(6) = l * j * ja;
  h.rightCols(6) = l * j * jb;
  Eigen::MatrixXd p(12, 12);
  try {
    gtsam::NonlinearFactorGraph live;
    for (const auto &factor : isam_->getFactorsUnsafe())
      if (factor)
        live.push_back(factor);
    const gtsam::Marginals marginals(live, values);
    const auto joint = marginals.jointMarginalCovariance({m.from, m.to});
    p.topLeftCorner(6, 6) = joint.at(m.from, m.from);
    p.topRightCorner(6, 6) = joint.at(m.from, m.to);
    p.bottomLeftCorner(6, 6) = joint.at(m.to, m.from);
    p.bottomRightCorner(6, 6) = joint.at(m.to, m.to);
  } catch (const std::exception &) {
    return out;
  }
  Eigen::MatrixXd s = Eigen::MatrixXd::Identity(l.rows(), l.rows()) + h * p * h.transpose();
  s = .5 * (s + s.transpose()).eval();
  Eigen::LDLT<Eigen::MatrixXd> ldlt(s);
  if (ldlt.info() != Eigen::Success || (ldlt.vectorD().array() <= 1e-12).any())
    return out;
  out.nis = std::max(0., e.dot(ldlt.solve(e)));
  out.dof = int(l.rows());
  try {
    out.probability = boost::math::cdf(boost::math::complement(
        boost::math::chi_squared(double(out.dof)), out.nis));
  } catch (const std::exception &) {
    return out;
  }
  out.valid = std::isfinite(out.nis) && std::isfinite(out.probability);
  return out;
}
/// 接收已验收的相对回环测量，并通过鲁棒因子更新两个活动端点。
/// 拒绝同端点、过期节点及重复测量，避免错误连接历史段或重复计权。
bool PoseGraph::addLoop(const LoopMeasurement &m) {
  if (m.segment != segment_ || m.measurement_id.empty() ||
      m.measurement_id.size() > 256 || observations_.size() >= 1024 ||
      observations_.count(m.measurement_id) || m.from == m.to ||
      !contains(m.from) || !contains(m.to))
    return false;
  if (!validPose(m.relative))
    throw std::invalid_argument("invalid loop pose");
  gtsam::NonlinearFactorGraph g;
  g.emplace_shared<LoopFactor>(m.from, m.to, m.relative,
                               information(m.sqrt_information),
                               config_.loop_cauchy_size);
  submit(g, gtsam::Values{});
  observations_[m.measurement_id] = {m.from, m.to};
  return true;
}
/// 读取指定节点的当前位姿估计。
Pose PoseGraph::estimate(Key k) const {
  return fromGtsam(isam_->calculateEstimate<gtsam::Pose3>(k));
}
/// 查询边缘协方差，换回 [平移, 旋转] 顺序并消除舍入导致的轻微非对称。
Covariance PoseGraph::covariance(Key k) const {
  Covariance p = reorder(isam_->marginalCovariance(k));
  return .5 * (p + p.transpose()).eval();
}
/// 按活动时间表取得节点位姿快照，不包含已边缘化节点；一次整体回代，避免逐节点求解。
std::map<Key, Pose> PoseGraph::estimates() const {
  std::map<Key, Pose> out;
  const gtsam::Values values = isam_->calculateEstimate();
  for (const auto &k : times_)
    out.emplace_hint(out.end(), k.first, fromGtsam(values.at<gtsam::Pose3>(k.first)));
  return out;
}
/// 读取因子容器总槽数，用于检查空槽积累。
std::size_t PoseGraph::factorSlots() const {
  return isam_->getFactorsUnsafe().size();
}
/// 查找活动节点时间戳，未知或已过期节点抛出异常。
std::int64_t PoseGraph::stamp(Key key) const {
  for (const auto &item : times_)
    if (item.first == key)
      return item.second;
  throw std::out_of_range("expired graph timestamp");
}
} // 命名空间 rm27_backend
