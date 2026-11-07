#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "sensor_polling_rate.hpp"

namespace {
std::atomic<long> realtime_offset_seconds{0};
}  // namespace

// Interpose only inside this test process, including librclcpp's clock reads.
// The kernel clock and every other process remain untouched. An adjustable
// realtime deadline can otherwise block ingestion beyond the IMU queue budget.
extern "C" int clock_gettime(clockid_t clock, struct timespec * stamp) noexcept
{
  const int result = static_cast<int>(syscall(SYS_clock_gettime, clock, stamp));
  if (result == 0 && clock == CLOCK_REALTIME) {
    stamp->tv_sec += realtime_offset_seconds.load(std::memory_order_relaxed);
  }
  return result;
}

TEST(SensorPollingClock, HostRealtimeCorrectionsCannotSuspendInputPolling)
{
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  point_lio::SensorPollingRate rate(500);
  std::atomic<bool> observed_adjustment{false};
  std::thread correction([&] {
    std::this_thread::sleep_for(20ms);
    realtime_offset_seconds.store(1);
    observed_adjustment.store(true);
    std::this_thread::sleep_for(40ms);
    realtime_offset_seconds.store(0);
    std::this_thread::sleep_for(40ms);
    realtime_offset_seconds.store(-1);
    std::this_thread::sleep_for(40ms);
    realtime_offset_seconds.store(0);
  });
  const auto started = std::chrono::steady_clock::now();
  auto previous = started;
  double maximum_gap = 0.0;
  size_t polls = 0;
  while (std::chrono::steady_clock::now() - started < 250ms) {
    rate.sleep();
    const auto now = std::chrono::steady_clock::now();
    maximum_gap = std::max(maximum_gap, std::chrono::duration<double>(now - previous).count());
    previous = now;
    ++polls;
  }
  correction.join();
  rclcpp::shutdown();
  ASSERT_TRUE(observed_adjustment.load());
  EXPECT_GT(polls, 20u);
  // 100 samples at 200 Hz cover 0.5 sensor seconds. Keep the scheduler well
  // inside that budget rather than increasing queues or weakening freshness.
  EXPECT_LT(maximum_gap, 0.2);
}
