/**
 * @file input.hpp
 * @brief 定义保留原始回波的有界观测窗口。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "zlio/config.hpp"

namespace zlio {

/// 表示输入中断或历史缺失使当前估计会话无法继续连续传播。
class InputDiscontinuity : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// 拥有有序原始样本，按时长、数量及水位释放历史并保留必要插值左端点。
class ImuHistory {
public:
    /// 验证并设置历史容量及连续采样间隔上限。
    explicit ImuHistory(ImuHistoryLimits limits = {});
    /// 追加有限且时间连续的样本并限制容量；重复时间戳返回 false，回退报告输入中断。
    bool push(const Imu& sample);
    /// 判断指定时刻是否处于可查询的历史范围。
    bool covers(Time time) const;
    /// 查询水位及其后的测量；缺少覆盖时抛出异常，水位本身可以插值。
    Imu at(Time time) const;
    /// 声明不再需要早于水位的测量，保留该水位的原始插值区间。
    void discardBefore(Time watermark);
    /// 判断两时刻间隔是否超过历史时长上限，安全处理整数边界。
    bool exceedsDelay(Time older, Time newer) const;
    /// 返回可接受查询的最早时刻，尚无样本时返回空值。
    std::optional<Time> minimumTime() const { return minimum_time_; }
    /// 借用原始样本序列，只读引用有效期截至下一次历史变更。
    const std::deque<Imu>& samples() const { return samples_; }

private:
    /// 计算给定时长对应的最早允许时刻，并对纳秒下界作饱和处理。
    static Time cutoff(Time newest, Time duration);
    /// 删除水位前冗余样本，并按数量限制继续推进水位。
    void trim();
    Time maximum_duration_, maximum_interval_;
    std::size_t maximum_samples_;
    std::optional<Time> minimum_time_;
    std::deque<Imu> samples_;
};

/// 保存窗口内保留真实时间和唯一编号的原始回波。
struct WindowPoint {
    Point point;
};

/// 借用有序回波的一段连续区间，不拥有底层存储。
class WindowPointRange {
public:
    using Iterator = std::vector<WindowPoint>::const_iterator;
    /// 用首尾迭代器描述左闭右开的存储区间。
    WindowPointRange(Iterator first, Iterator last) : first_(first), last_(last) {}
    /// 返回区间首个回波的迭代器。
    Iterator begin() const { return first_; }
    /// 返回区间末尾之后的迭代器。
    Iterator end() const { return last_; }
    /// 返回区间包含的回波数量。
    std::size_t size() const { return static_cast<std::size_t>(last_ - first_); }
    /// 按相对索引读取回波，调用方保证索引有效。
    const WindowPoint& operator[](std::size_t index) const { return first_[index]; }

private:
    Iterator first_, last_;
};

/// 保存原始回波和连续调度端点，不参与导航状态估计。
class ReturnBuffer {
public:
    /// 按最大更新间隔设置回波历史长度。
    explicit ReturnBuffer(WindowConfig config = {});
    /// 从同步边界开始安排首个完整窗口。
    void start(Time time);
    /// 为新回波分配编号并按时间稳定合并到缓冲区。
    void push(Scan scan);
    /// 返回下一次计划处理的窗口末端。
    Time nextEnd() const { return next_end_; }
    /// 返回已接收回波的最大时间戳。
    Time latestPoint() const { return buffer_.latestTime(); }
    /// 返回尚未淘汰的回波数量。
    std::size_t size() const { return buffer_.size(); }
    /// 借用 (end-length, end]；下次追加或前移后不得继续持有。
    WindowPointRange pointRange(Time end) const;
    /// 安排下一端点并释放超出最大观测跨度的历史回波。
    void advance(Time end, double interval);

private:
    /// 拥有连续回波存储、唯一编号和有效区间起点。
    class PointBuffer {
    public:
        /// 追加有序回波，对重叠或迟到区间执行稳定归并。
        void push(Scan scan);
        /// 返回真实时间位于 (begin, end] 的借用区间。
        WindowPointRange range(Time begin, Time end) const;
        /// 淘汰时间小于等于给定边界的回波。
        void discardThrough(Time time);
        /// 返回已接收回波的最大时间戳。
        Time latestTime() const { return latest_time_; }
        /// 返回有效区间内的回波数量。
        std::size_t size() const { return points_.size() - first_point_; }

    private:
        std::vector<WindowPoint> points_;
        std::size_t first_point_ = 0;
        std::uint64_t next_id_ = 0;
        Time latest_time_ = 0;
    };

    PointBuffer buffer_;
    Time length_, next_end_ = 0;
};

/// 配置原始测量历史和回波缓冲容量。
struct InputBufferLimits {
    ImuHistoryLimits imu_history;
    std::size_t maximum_lidar_points = 1000000;
};

/// 保存一个待处理的启动扫描。
struct StartupScan {
    Scan points;
    Time end;
};

/// 管理原始测量、传输包边界及已消费时间。
class Input {
public:
    /// 设置传感器量程和测量缓冲容量。
    Input(const Config& config, InputBufferLimits limits);
    /// 保存原始惯性测量并检查处理水位，重复时间戳返回 false。
    bool addImu(const Imu& sample);
    /// 筛选量程内回波并保存输入包。
    void addScan(Scan scan);
    /// 提取已有完整惯性覆盖的启动扫描。
    std::optional<StartupScan> takeStartup();
    /// 取得相邻启动边界之间的惯性测量。
    std::vector<Imu> startupImus(Time end) const;
    /// 推进启动处理边界并回收惯性历史。
    void advanceStartup(Time end);
    /// 安装运行起点并转交剩余扫描。
    void start(Time end);
    /// 清空回波及调度历史，仅保留运行边界插值及后续传播所需的原始 IMU。
    void restartTracking(Time end);
    /// 返回当前可处理的下一个观测端点。
    std::optional<Time> nextEnd(bool flush);
    /// 提交观测端点及下一次期望间隔。
    void advance(Time end, double interval);
    /// 返回原始惯性历史。
    const ImuHistory& imus() const { return imus_; }
    /// 返回采样反馈所需的回波范围。
    WindowPointRange points(Time end) const { return points_.pointRange(end); }
    /// 返回最后一个已消费的观测端点。
    Time consumed() const { return consumed_; }
    /// 返回当前启动边界。
    std::optional<Time> startupBoundary() const { return boundary_; }

private:
    /// 记录运行期输入包并转交回波缓冲。
    void pushTracking(Scan scan);
    ImuHistory imus_;
    ReturnBuffer points_;
    WindowConfig window_;
    double minimum_range_, maximum_range_;
    std::size_t maximum_points_, startup_points_ = 0;
    std::deque<StartupScan> startup_;
    std::deque<std::pair<Time, Time>> packets_;
    std::optional<Time> boundary_;
    Time consumed_ = 0;
    bool tracking_ = false;
};

}  // namespace zlio
