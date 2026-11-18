#pragma once

#include "rm27_backend/types.hpp"
#include <chrono>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>

namespace rm27_backend {
/// 等待同刻机械 TF 的短队列；不持有点云，不跨身份、图段或定位代次补发。
template <class Payload> class NavigationOutputQueue {
public:
  using Clock = std::chrono::steady_clock;
  struct Context {
    Identity identity;
    std::uint64_t segment = 0, generation = 0;
    bool operator==(const Context &other) const {
      return identity == other.identity && segment == other.segment &&
             generation == other.generation;
    }
  };
  struct Sample {
    Frame frame;
    Pose estimate;
    Payload payload;
    Clock::time_point received;
  };

  explicit NavigationOutputQueue(std::size_t capacity = 8, double max_age = .5)
      : capacity_(capacity), max_age_(max_age) {
    if (capacity == 0 || !std::isfinite(max_age) || max_age <= 0)
      throw std::invalid_argument("invalid navigation output queue limits");
  }
  void clear() {
    samples_.clear();
    context_.reset();
    last_stamp_ = -1;
    last_sequence_ = 0;
  }
  void synchronize(const Context &context) {
    if (!context_ || !(*context_ == context)) {
      clear();
      context_ = context;
    }
  }
  bool push(const Frame &frame, const Pose &estimate, Payload payload,
            Clock::time_point received) {
    if (!context_ || frame.identity != context_->identity ||
        frame.stamp_ns <= last_stamp_ || frame.sequence <= last_sequence_)
      return false;
    last_stamp_ = frame.stamp_ns;
    last_sequence_ = frame.sequence;
    if (samples_.size() == capacity_) {
      samples_.pop_front();
      ++overflow_;
    }
    samples_.push_back({frame, estimate, std::move(payload), received});
    return true;
  }
  /// 选择最新的可发布帧，跳过尚无 TF 的新帧；保持原始时间戳与完整载荷配对。
  /// watermark 包括冻结/预览 TF，旧估计不能使已发布的时间倒退。
  template <class Publish>
  bool publishNewestReady(Clock::time_point now, std::int64_t latest_stamp,
                          std::int64_t watermark, Publish publish) {
    while (!samples_.empty()) {
      const auto &front = samples_.front();
      if (front.frame.stamp_ns > watermark &&
          fresh(front.received, front.frame.stamp_ns, now, latest_stamp))
        break;
      samples_.pop_front();
      ++expired_;
    }
    for (std::size_t i = samples_.size(); i > 0; --i) {
      const auto &sample = samples_[i - 1];
      if (fresh(sample.received, sample.frame.stamp_ns, now, latest_stamp) &&
          publish(sample)) {
        samples_.erase(samples_.begin(), samples_.begin() + i);
        return true;
      }
    }
    return false;
  }
  bool fresh(Clock::time_point received, std::int64_t stamp,
             Clock::time_point now, std::int64_t latest_stamp) const {
    const double wall_age =
        std::chrono::duration<double>(now - received).count();
    const double source_age = (latest_stamp - stamp) * 1e-9;
    return wall_age >= 0 && wall_age <= max_age_ && source_age >= 0 &&
           source_age <= max_age_;
  }
  bool matches(const Context &context) const {
    return context_ && *context_ == context;
  }
  std::size_t size() const { return samples_.size(); }
  std::uint64_t expired() const { return expired_; }
  std::uint64_t overflow() const { return overflow_; }

private:
  std::size_t capacity_;
  double max_age_;
  std::optional<Context> context_;
  std::deque<Sample> samples_;
  std::int64_t last_stamp_ = -1;
  std::uint64_t last_sequence_ = 0, expired_ = 0, overflow_ = 0;
};
} // namespace rm27_backend
