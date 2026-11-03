/** @file input.cpp @brief 保存回波真实时刻，按连续端点推进有界窗口。 */
#include "zlio/input.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "zlio/inertial.hpp"

namespace zlio {
namespace {
/// 将有限正时长转换为可表示的整数纳秒。
Time checkedDuration(double duration) {
    // 整秒上界向下取整，为纳秒转换保留可表示范围。
    constexpr Time kMaximumWholeSeconds = std::numeric_limits<Time>::max() / 1000000000LL;
    if (!std::isfinite(duration) || duration <= 0 ||
        duration > static_cast<double>(kMaximumWholeSeconds))
        throw std::invalid_argument(
            "IMU duration limit must be finite, positive and representable");
    const Time result = nanoseconds(duration);
    if (result <= 0)
        throw std::invalid_argument("IMU duration limit must be at least one nanosecond");
    return result;
}
}  // namespace

/// 设置历史保留时长、连续采样间隔和样本容量。
ImuHistory::ImuHistory(ImuHistoryLimits limits)
    : maximum_duration_(checkedDuration(limits.maximum_duration)),
      maximum_interval_(checkedDuration(limits.maximum_interval)),
      maximum_samples_(limits.maximum_samples) {
    if (maximum_samples_ < 2)
        throw std::invalid_argument("IMU history must retain at least two samples");
}

/// 计算指定时长对应的时刻下界，避免整数减法下溢。
Time ImuHistory::cutoff(Time newest, Time duration) {
    const Time minimum = std::numeric_limits<Time>::min();
    return newest < minimum + duration ? minimum : newest - duration;
}

/// 判断旧时刻是否严格早于新时刻的可接受历史范围。
bool ImuHistory::exceedsDelay(Time older, Time newer) const {
    return older < cutoff(newer, maximum_duration_);
}

/// 查询时刻须同时满足水位下界和最新样本上界。
bool ImuHistory::covers(Time time) const {
    return !samples_.empty() && time >= *minimum_time_ && time <= samples_.back().time;
}

/// 按水位和样本数量清退历史，保留水位的插值左端样本。
void ImuHistory::trim() {
    while (samples_.size() > 1 && samples_[1].time <= *minimum_time_)
        samples_.pop_front();
    while (samples_.size() > maximum_samples_) {
        samples_.pop_front();
        minimum_time_ = std::max(*minimum_time_, samples_.front().time);
    }
}

/// 验证有限测量并忽略重复时间戳，回退或断流时保留原历史并报告输入中断。
bool ImuHistory::push(const Imu& sample) {
    if (!sample.acceleration.allFinite() || !sample.angular_velocity.allFinite())
        throw std::invalid_argument("IMU measurements must be finite");
    if (!samples_.empty()) {
        const Time previous = samples_.back().time;
        if (sample.time == previous)
            return false;
        if (sample.time < previous)
            throw InputDiscontinuity("IMU timestamp moved backwards");
        if (previous < cutoff(sample.time, maximum_interval_))
            throw InputDiscontinuity("IMU sampling gap exceeds the maximum interval");
    }
    samples_.push_back(sample);
    minimum_time_ =
        std::max(minimum_time_.value_or(sample.time), cutoff(sample.time, maximum_duration_));
    trim();
    return true;
}

/// 依据调用方承诺推进历史水位，拒绝超出当前样本覆盖的水位。
void ImuHistory::discardBefore(Time watermark) {
    if (samples_.empty() || watermark > samples_.back().time)
        throw std::out_of_range("IMU history watermark is outside the available time range");
    minimum_time_ = std::max(*minimum_time_, watermark);
    trim();
}

/// 在保留范围内获取原始测量或保留区间身份的线性插值测量。
Imu ImuHistory::at(Time time) const {
    if (minimum_time_ && time < *minimum_time_)
        throw std::out_of_range("IMU query time precedes the history watermark");
    const auto right =
        std::lower_bound(samples_.begin(), samples_.end(), time,
                         [](const Imu& sample, Time stamp) { return sample.time < stamp; });
    if (right != samples_.end() && right->time == time)
        return *right;
    if (right == samples_.begin() || right == samples_.end())
        throw std::out_of_range("IMU measurements do not cover the query time");
    return interpolateImu(*std::prev(right), *right, time);
}

/// 配置回波历史保留跨度。
ReturnBuffer::ReturnBuffer(WindowConfig config) : length_(nanoseconds(config.maximum_interval)) {
}
/// 设置首次观测的期望端点。
void ReturnBuffer::start(Time time) {
    next_end_ = time + length_;
}
/// 保存原始回波并维持时间顺序。
void ReturnBuffer::push(Scan scan) {
    buffer_.push(std::move(scan));
}
/// 返回观测端点之前的有效回波范围。
WindowPointRange ReturnBuffer::pointRange(Time end) const {
    return buffer_.range(end - length_, end);
}
/// 推进观测端点并回收过期回波。
void ReturnBuffer::advance(Time end, double interval) {
    if (!std::isfinite(interval) || interval <= 0 || interval > seconds(length_) + 1e-9)
        throw std::invalid_argument(
            "observation step must be finite, positive and within the maximum interval");
    const Time step = nanoseconds(interval);
    if (step <= 0 || step > length_)
        throw std::invalid_argument(
            "observation step must be positive in nanoseconds and within the maximum interval");
    next_end_ = end + step;
    buffer_.discardThrough(end - length_);
}

/// 分配回波身份并稳定合并有序输入。
void ReturnBuffer::PointBuffer::push(Scan scan) {
    if (scan.empty())
        return;
    const auto earlier = [](const Point& a, const Point& b) { return a.time < b.time; };
    if (!std::is_sorted(scan.begin(), scan.end(), earlier))
        std::stable_sort(scan.begin(), scan.end(), earlier);
    // 按需回收已过期前缀；避免每次窗口前移都搬动存活点。
    if (first_point_ && (first_point_ >= points_.size() - first_point_ ||
                         scan.size() > points_.capacity() - points_.size())) {
        points_.erase(points_.begin(), points_.begin() + first_point_);
        first_point_ = 0;
    }
    if (scan.size() > points_.capacity() - points_.size())
        points_.reserve(points_.size() + std::max(points_.size(), scan.size()));
    const std::size_t old_size = points_.size();
    const Time first_time = scan.front().time;
    const bool overlaps = old_size > first_point_ && points_.back().point.time > first_time;
    for (auto& point : scan) {
        latest_time_ = std::max(latest_time_, point.time);
        point.id = next_id_++;
        points_.push_back({std::move(point)});
    }
    if (overlaps) {
        const auto middle = points_.begin() + old_size;
        // 同时刻旧点保持在新点之前，仅归并真正重叠的旧后缀。
        const auto first = std::upper_bound(
            points_.begin() + first_point_, middle, first_time,
            [](Time time, const WindowPoint& point) { return time < point.point.time; });
        std::inplace_merge(
            first, middle, points_.end(),
            [](const WindowPoint& a, const WindowPoint& b) { return a.point.time < b.point.time; });
    }
}

/// 返回时间位于左开右闭区间内的连续回波。
WindowPointRange ReturnBuffer::PointBuffer::range(Time begin, Time end) const {
    const auto earlier = [](Time time, const WindowPoint& point) {
        return time < point.point.time;
    };
    const auto first =
        std::upper_bound(points_.begin() + first_point_, points_.end(), begin, earlier);
    return {first, std::upper_bound(first, points_.end(), end, earlier)};
}

/// 回收给定时刻及其之前的回波。
void ReturnBuffer::PointBuffer::discardThrough(Time time) {
    const auto first = std::upper_bound(
        points_.begin() + first_point_, points_.end(), time,
        [](Time time, const WindowPoint& point) { return time < point.point.time; });
    first_point_ = static_cast<std::size_t>(first - points_.begin());
    if (first_point_ == points_.size()) {
        points_.clear();
        first_point_ = 0;
    }
}

namespace {
/// 在有限输入区间中安排观测端点并保留有效尾段。
Time packetStep(Time remaining, Time requested, Time minimum, Time maximum) {
    if (remaining <= 0 || minimum <= 0 || maximum < 2 * minimum)
        throw std::invalid_argument("invalid input interval or observation step");
    Time result = std::min(remaining, std::clamp(requested, minimum, maximum));
    const Time tail = remaining - result;
    if (tail > 0 && tail < minimum)
        result = remaining >= 2 * minimum ? remaining - minimum : remaining;
    return result;
}
}  // namespace

/// 配置输入量程、时间跨度和缓存容量。
Input::Input(const Config& config, InputBufferLimits limits)
    : imus_(limits.imu_history),
      points_(config.window),
      window_(config.window),
      minimum_range_(config.minimum_range),
      maximum_range_(config.maximum_range),
      maximum_points_(limits.maximum_lidar_points) {
    if (maximum_points_ == 0)
        throw std::invalid_argument("input cloud capacity must be positive");
}

/// 追加新的惯性测量并校验已处理时间的覆盖，重复输入不推进处理。
bool Input::addImu(const Imu& sample) {
    if (!imus_.push(sample))
        return false;
    const auto watermark = tracking_ ? std::optional<Time>(consumed_) : boundary_;
    if (watermark && *watermark < *imus_.minimumTime())
        throw InputDiscontinuity(
            "LiDAR processing time precedes the retained IMU history watermark");
    return true;
}

/// 筛选回波并按当前处理阶段保存输入包。
void Input::addScan(Scan scan) {
    if (scan.size() > maximum_points_)
        throw std::length_error("LiDAR packet input point limit exceeded");
    std::optional<Time> acquisition_end;
    if (!scan.empty())
        acquisition_end =
            std::max_element(scan.begin(), scan.end(), [](const Point& a, const Point& b) {
                return a.time < b.time;
            })->time;
    scan.erase(std::remove_if(scan.begin(), scan.end(),
                              [&](const Point& point) {
                                  const double range = point.position.norm();
                                  return !point.position.allFinite() || range < minimum_range_ ||
                                         range > maximum_range_;
                              }),
               scan.end());

    const auto& samples = imus_.samples();
    if (acquisition_end && !samples.empty() &&
        (imus_.exceedsDelay(samples.back().time, *acquisition_end) ||
         imus_.exceedsDelay(*acquisition_end, samples.back().time)))
        throw InputDiscontinuity("LiDAR and IMU time difference exceeds the allowed input latency");
    if (acquisition_end && !startup_.empty() &&
        imus_.exceedsDelay(startup_.front().end, *acquisition_end))
        throw InputDiscontinuity("initialization cloud exceeded the allowed wait for IMU data");
    const auto buffered = tracking_ ? points_.size() : startup_points_;
    if (scan.size() > maximum_points_ - buffered) {
        if (samples.empty() || (acquisition_end && samples.back().time < *acquisition_end))
            throw InputDiscontinuity("LiDAR buffer exhausted while waiting for IMU coverage");
        throw std::length_error("LiDAR input buffer point limit exceeded");
    }
    if (tracking_) {
        pushTracking(std::move(scan));
    } else if (!scan.empty()) {
        startup_points_ += scan.size();
        startup_.push_back({std::move(scan), *acquisition_end});
    }
}

/// 提取最早的完整启动输入包。
std::optional<StartupScan> Input::takeStartup() {
    if (startup_.empty() || imus_.samples().empty() ||
        imus_.samples().back().time < startup_.front().end)
        return std::nullopt;
    auto packet = std::move(startup_.front());
    startup_.pop_front();
    startup_points_ -= packet.points.size();
    return packet;
}

/// 构造相邻启动边界之间的完整惯性区间。
std::vector<Imu> Input::startupImus(Time end) const {
    const Time begin = *boundary_;
    std::vector<Imu> bundle{imus_.at(begin)};
    for (const auto& imu : imus_.samples())
        if (imu.time > begin && imu.time < end)
            bundle.push_back(imu);
    bundle.push_back(imus_.at(end));
    return bundle;
}

/// 提交启动扫描边界并回收不再需要的惯性样本。
void Input::advanceStartup(Time end) {
    boundary_ = end;
    imus_.discardBefore(end);
}

/// 将尚未处理的输入包转入运行缓冲。
void Input::start(Time end) {
    tracking_ = true;
    consumed_ = end;
    points_.start(end);
    for (auto& pending : startup_)
        pushTracking(std::move(pending.points));
    startup_.clear();
    startup_points_ = 0;
    advanceStartup(end);
}
/// 在已有惯性支撑上建立新的回波消费和调度边界。
void Input::restartTracking(Time end) {
    // 先确认后验时刻仍有惯性历史覆盖，再建立新的处理边界。
    (void)imus_.at(end);
    imus_.discardBefore(end);
    points_ = ReturnBuffer(window_);
    packets_.clear();
    startup_.clear();
    startup_points_ = 0;
    tracking_ = true;
    consumed_ = end;
    boundary_ = end;
    points_.start(end);
}

/// 保存有效输入包的首尾时刻和回波。
void Input::pushTracking(Scan scan) {
    if (!scan.empty()) {
        const auto ends =
            std::minmax_element(scan.begin(), scan.end(),
                                [](const Point& a, const Point& b) { return a.time < b.time; });
        packets_.emplace_back(ends.first->time, ends.second->time);
    }
    points_.push(std::move(scan));
}

/// 根据已到齐数据和期望步长选择下一观测端点。
std::optional<Time> Input::nextEnd(bool flush) {
    if (!tracking_ || imus_.samples().empty())
        return std::nullopt;
    const Time available = std::min(imus_.samples().back().time, points_.latestPoint());
    const Time minimum = nanoseconds(window_.minimum_interval);
    const Time maximum = nanoseconds(window_.maximum_interval);
    while (!packets_.empty()) {
        if (packets_.front().second <= consumed_) {
            packets_.pop_front();
            continue;
        }
        const auto packet = packets_.front();
        if (packet.second - packet.first < minimum) {
            if (points_.nextEnd() <= available)
                return points_.nextEnd();
            break;
        }
        if (packet.second > available)
            break;
        return consumed_ + packetStep(packet.second - consumed_, points_.nextEnd() - consumed_,
                                      minimum, maximum);
    }
    if (flush && available > consumed_)
        return available;
    return std::nullopt;
}

/// 推进观测水位并回收过期传感器历史。
void Input::advance(Time end, double interval) {
    points_.advance(end, interval);
    consumed_ = end;
    imus_.discardBefore(end - nanoseconds(window_.maximum_interval));
}

}  // namespace zlio
