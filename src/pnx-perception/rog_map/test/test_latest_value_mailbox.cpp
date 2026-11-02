#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <rog_map_ros/latest_value_mailbox.hpp>
#include <thread>

namespace
{

using Mailbox = rog_map::detail::LatestValueMailbox<int>;

TEST(LatestValueMailbox, ReturnsLatestValueAndCoalescedCount)
{
  Mailbox mailbox;

  EXPECT_FALSE(mailbox.takeLatest().has_value());
  mailbox.store(10);
  mailbox.store(20);

  const auto delivery = mailbox.takeLatest();
  ASSERT_TRUE(delivery.has_value());
  EXPECT_EQ(delivery->value, 20);
  EXPECT_EQ(delivery->pending_count, 2U);
  EXPECT_FALSE(mailbox.takeLatest().has_value());
}

TEST(LatestValueMailbox, ConcurrentStoreAndTakeDoNotLosePendingCounts)
{
  constexpr int kFrameCount = 20000;
  Mailbox mailbox;
  std::atomic<bool> producer_done{false};
  std::atomic<std::size_t> consumed_count{0U};

  std::thread producer([&]() {
    for (int frame = 1; frame <= kFrameCount; ++frame) {
      mailbox.store(frame);
      if ((frame % 64) == 0) {
        std::this_thread::yield();
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    while (!producer_done.load(std::memory_order_acquire)) {
      if (const auto delivery = mailbox.takeLatest()) {
        consumed_count.fetch_add(delivery->pending_count, std::memory_order_relaxed);
      } else {
        std::this_thread::yield();
      }
    }
  });

  producer.join();
  consumer.join();
  if (const auto delivery = mailbox.takeLatest()) {
    consumed_count.fetch_add(delivery->pending_count, std::memory_order_relaxed);
  }

  EXPECT_EQ(consumed_count.load(std::memory_order_relaxed), static_cast<std::size_t>(kFrameCount));
  EXPECT_FALSE(mailbox.takeLatest().has_value());
}

}  // namespace
