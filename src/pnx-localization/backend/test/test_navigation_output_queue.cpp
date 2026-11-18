#include "rm27_backend/navigation_output_queue.hpp"
#include <gtest/gtest.h>
#include <vector>

namespace rm27_backend {
namespace {
using Queue = NavigationOutputQueue<std::int64_t>;
using namespace std::chrono_literals;
const Queue::Context context{{"frontend", 0}, 1, 2};
const auto start = Queue::Clock::time_point(10s);
Frame frame(int sequence, std::int64_t epoch = 0) {
  Frame f;
  f.identity = {"frontend", std::uint64_t(epoch)};
  f.sequence = sequence;
  f.stamp_ns = sequence * 100000000LL;
  return f;
}

TEST(NavigationOutputQueue,
     OneScanLateTfStillPublishesPreviousFrameWithItsPayload) {
  Queue queue;
  queue.synchronize(context);
  std::int64_t watermark = -1;
  std::vector<std::int64_t> published;
  for (int i = 1; i <= 20; ++i) {
    const auto f = frame(i);
    const auto now = start + i * 100ms;
    ASSERT_TRUE(queue.push(f, f.odom, f.stamp_ns, now));
    const bool ready = queue.publishNewestReady(
        now, f.stamp_ns, watermark, [&](const auto &sample) {
          if (sample.frame.sequence >= f.sequence)
            return false; // 底盘 TF 始终比原子帧晚一帧。
          EXPECT_EQ(sample.payload, sample.frame.stamp_ns);
          EXPECT_GT(sample.frame.stamp_ns, watermark);
          watermark = sample.frame.stamp_ns;
          published.push_back(watermark);
          return true;
        });
    EXPECT_EQ(ready, i > 1);
    EXPECT_EQ(queue.size(), 1U);
  }
  ASSERT_EQ(published.size(), 19U);
  EXPECT_EQ(published.back(), frame(19).stamp_ns);
}

TEST(NavigationOutputQueue,
     RetryWithoutNewInputChoosesNewestReadyAndNeverRewinds) {
  Queue queue;
  queue.synchronize(context);
  for (int i = 1; i <= 3; ++i)
    ASSERT_TRUE(queue.push(frame(i), Pose::Identity(), i, start));
  std::vector<int> published;
  EXPECT_TRUE(queue.publishNewestReady(start + 20ms, frame(3).stamp_ns, -1,
                                       [&](const auto &s) {
                                         published.push_back(s.payload);
                                         return true;
                                       }));
  EXPECT_EQ(published, std::vector<int>{3});
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_FALSE(queue.push(frame(2), Pose::Identity(), 2, start + 30ms));
  ASSERT_TRUE(queue.push(frame(4), Pose::Identity(), 4, start + 40ms));
  // 人工预览已推进 TF 水位，不能补发覆盖其之前的输出。
  EXPECT_FALSE(queue.publishNewestReady(
      start + 50ms, frame(4).stamp_ns, frame(4).stamp_ns + 1, [](const auto &) {
        ADD_FAILURE();
        return true;
      }));
}

TEST(NavigationOutputQueue,
     IdentitySegmentAndGenerationChangesDiscardWaitingWork) {
  for (int change = 0; change < 4; ++change) {
    Queue queue;
    queue.synchronize(context);
    ASSERT_TRUE(queue.push(frame(1), Pose::Identity(), 1, start));
    auto next = context;
    if (change == 0)
      ++next.identity.epoch;
    if (change == 1)
      next.identity.instance = "replacement";
    if (change == 2)
      ++next.segment;
    if (change == 3)
      ++next.generation;
    queue.synchronize(next);
    EXPECT_EQ(queue.size(), 0U);
    EXPECT_FALSE(queue.matches(context));
    auto fresh = frame(1);
    fresh.identity = next.identity;
    EXPECT_TRUE(queue.push(fresh, Pose::Identity(), 2, start));
  }
}

TEST(NavigationOutputQueue, CapacityWallAndSourceAgeAreBounded) {
  Queue queue(2);
  queue.synchronize(context);
  for (int i = 1; i <= 3; ++i)
    queue.push(frame(i), Pose::Identity(), i, start);
  EXPECT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.overflow(), 1U);
  EXPECT_FALSE(queue.publishNewestReady(start + 501ms, frame(3).stamp_ns, -1,
                                        [](const auto &) {
                                          ADD_FAILURE();
                                          return true;
                                        }));
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_EQ(queue.expired(), 2U);
  queue.clear();
  queue.synchronize(context);
  queue.push(frame(1), Pose::Identity(), 1, start);
  EXPECT_FALSE(
      queue.publishNewestReady(start, frame(7).stamp_ns, -1, [](const auto &) {
        ADD_FAILURE();
        return true;
      }));
  EXPECT_FALSE(queue.fresh(start, frame(2).stamp_ns, start, frame(1).stamp_ns));
}

TEST(NavigationOutputQueue,
     ClearDuringResetPreventsLateTfFromRevivingOldOutput) {
  Queue queue;
  queue.synchronize(context);
  queue.push(frame(1), Pose::Identity(), 1, start);
  queue.clear();
  EXPECT_FALSE(queue.publishNewestReady(start + 20ms, frame(1).stamp_ns, -1,
                                        [](const auto &) {
                                          ADD_FAILURE();
                                          return true;
                                        }));
  EXPECT_FALSE(queue.matches(context));
}
} // namespace
} // namespace rm27_backend
