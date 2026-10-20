// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/raw_scan_worker.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <future>
#include <vector>
using namespace global_relocalization;
namespace {
using Worker=RawScanWorker<int,int>;
template<class Predicate> bool until(Predicate predicate){const auto deadline=Worker::Clock::now()+std::chrono::seconds(2);while(!predicate()){if(Worker::Clock::now()>=deadline)return false;std::this_thread::sleep_for(std::chrono::milliseconds(1));}return true;}
}
TEST(RawScanWorker,DelayedContiguousBurstsDrainInOrderWithoutControlTicks) {
  for(const double delay:{.7,1.0}) {
    std::atomic<double> ros{12.};Worker worker([](const Worker::Item & item){std::this_thread::sleep_for(std::chrono::milliseconds(2));return item.payload;},[&]{return ros.load();});
    worker.begin();worker.watermark(12.);
    ASSERT_TRUE(worker.enqueue(-1,1024,10.9,10.9,Worker::Clock::now()-std::chrono::duration_cast<Worker::Clock::duration>(std::chrono::duration<double>(delay))));
    ASSERT_TRUE(until([&]{return worker.stats().result_depth==1;}));worker.takeResult();
    for(int j=0;j<10;++j)ASSERT_TRUE(worker.enqueue(j,1024,11.+j*.1,12.));
    ASSERT_TRUE(until([&]{return worker.stats().result_depth==10;}));
    auto results=worker.takeResults();ASSERT_EQ(results.size(),10u);
    for(int j=0;j<10;++j){EXPECT_EQ(results[j].result,j);EXPECT_NEAR(results[j].item.stamp,11.+j*.1,1e-12);EXPECT_EQ(results[j].item.payload,0);}
    EXPECT_TRUE(worker.stats().error.empty());EXPECT_EQ(worker.stats().outstanding,0u);EXPECT_GE(worker.stats().max_callback_gap,delay);
  }
}
TEST(RawScanWorker,LargeBurstAndUnconsumedResultsShareBoundedCapacity) {
  Worker worker([](const Worker::Item & item){return item.payload;},[]{return 20.;});worker.begin();worker.watermark(20.);
  for(int j=0;j<32;++j)ASSERT_TRUE(worker.enqueue(j,Worker::max_message_bytes,19.+j*.02,20.));
  ASSERT_TRUE(until([&]{return worker.stats().result_depth==32;}));
  EXPECT_EQ(worker.stats().bytes,32u*Worker::max_message_bytes);EXPECT_FALSE(worker.enqueue(33,1,19.64,20.));
  EXPECT_EQ(worker.stats().overflow_count,1u);EXPECT_NE(worker.stats().error.find("overflow"),std::string::npos);
  EXPECT_EQ(worker.stats().outstanding,32u);worker.cancel();EXPECT_EQ(worker.stats().outstanding,0u);
}
TEST(RawScanWorker,OversizeSourceGapAndReceiptAgeFailWithoutProcessing) {
  std::atomic<int> calls{0};Worker worker([&](const Worker::Item & item){++calls;return item.payload;},[]{return 20.;});
  worker.begin();EXPECT_FALSE(worker.enqueue(0,Worker::max_message_bytes+1,20.,20.));EXPECT_EQ(worker.stats().oversized_count,1u);
  worker.begin();ASSERT_TRUE(worker.enqueue(0,1,19.5,20.));EXPECT_FALSE(worker.enqueue(1,1,19.86,20.));EXPECT_NE(worker.stats().error.find("source scan coverage gap"),std::string::npos);
  worker.begin();EXPECT_FALSE(worker.enqueue(0,1,18.5,20.));EXPECT_EQ(worker.stats().stale_count,1u);EXPECT_EQ(calls,0);
}
TEST(RawScanWorker,WaitingForImuCoverageExpiresAndWorkerChecksCurrentAge) {
  std::atomic<double> ros{20.};std::atomic<int> calls{0};Worker worker([&](const Worker::Item & item){++calls;return item.payload;},[&]{return ros.load();});
  worker.begin();ASSERT_TRUE(worker.enqueue(0,1,20.,20.));ros.store(22.);
  ASSERT_TRUE(until([&]{return !worker.stats().error.empty();}));EXPECT_EQ(worker.stats().stale_count,1u);EXPECT_EQ(calls,0);
  worker.begin();ros.store(20.);ASSERT_TRUE(worker.enqueue(0,1,20.,20.));ros.store(22.);worker.watermark(22.);
  ASSERT_TRUE(until([&]{return !worker.stats().error.empty();}));EXPECT_EQ(worker.stats().stale_count,1u);EXPECT_EQ(calls,0);
}
TEST(RawScanWorker,CancelResetWaitForOldWorkAndNeverDeliverItsResult) {
  std::promise<void> entered,release;auto released=release.get_future().share();std::atomic<int> calls{0};
  Worker worker([&](const Worker::Item & item){if(++calls==1){entered.set_value();released.wait();}return item.payload;},[]{return 20.;});
  const auto old=worker.begin();worker.watermark(20.);ASSERT_TRUE(worker.enqueue(7,1,20.,20.));entered.get_future().wait();worker.cancel();
  auto begin=std::async(std::launch::async,[&]{return worker.begin();});EXPECT_EQ(begin.wait_for(std::chrono::milliseconds(20)),std::future_status::timeout);
  release.set_value();ASSERT_EQ(begin.wait_for(std::chrono::seconds(2)),std::future_status::ready);const auto current=begin.get();EXPECT_GT(current,old);
  EXPECT_TRUE(worker.takeResults().empty());worker.watermark(20.);ASSERT_TRUE(worker.enqueue(8,1,20.,20.));
  ASSERT_TRUE(until([&]{return worker.stats().result_depth==1;}));auto result=worker.takeResult();ASSERT_TRUE(result);EXPECT_EQ(result->result,8);EXPECT_EQ(result->item.generation,current);
}
TEST(RawScanWorker,FreezeWaitsForInFlightAndKeepsFinalOrderedResult) {
  std::promise<void> entered,release;auto released=release.get_future().share();std::atomic<int> calls{0};
  Worker worker([&](const Worker::Item & item){++calls;entered.set_value();released.wait();return item.payload;},[]{return 20.;});
  worker.begin();worker.watermark(20.);ASSERT_TRUE(worker.enqueue(7,1,19.9,20.));entered.get_future().wait();ASSERT_TRUE(worker.enqueue(8,1,20.,20.));
  auto frozen=std::async(std::launch::async,[&]{worker.freeze();});EXPECT_EQ(frozen.wait_for(std::chrono::milliseconds(20)),std::future_status::timeout);
  release.set_value();ASSERT_EQ(frozen.wait_for(std::chrono::seconds(2)),std::future_status::ready);frozen.get();
  auto result=worker.takeResult();ASSERT_TRUE(result);EXPECT_EQ(result->result,7);EXPECT_FALSE(worker.takeResult());EXPECT_EQ(worker.stats().ignored_count,1u);
  ASSERT_TRUE(worker.enqueue(9,1,20.1,20.1));EXPECT_EQ(calls,1);EXPECT_EQ(worker.stats().ignored_count,2u);
}

#include "global_relocalization/stopped_confirmation.hpp"
TEST(StoppedConfirmation,MotionFollowedByStableResultInSameBatchRestartsBothClocks) {
  using Clock=std::chrono::steady_clock;
  const auto origin=Clock::time_point{};
  auto wall_start=origin;double ros_start=10.;size_t confirmations=3;
  auto consume=[&](double linear,double angular,double wall,double ros,double stamp) {
    const auto state=observeStoppedScan(linear,angular,origin+std::chrono::milliseconds(int(wall*1000)),ros,stamp,2.,wall_start,ros_start);
    if(state==StoppedScanState::Moving)confirmations=0;
    else if(state==StoppedScanState::Ready)++confirmations;
    return state;
  };
  EXPECT_EQ(consume(.05,0,5.,15.,14.9),StoppedScanState::Moving);
  EXPECT_EQ(confirmations,0u);EXPECT_DOUBLE_EQ(ros_start,15.);
  EXPECT_EQ(consume(0,0,5.,15.,15.),StoppedScanState::Settling);
  EXPECT_EQ(confirmations,0u);
  EXPECT_EQ(consume(0,0,7.,17.,16.99),StoppedScanState::Settling);
  EXPECT_EQ(consume(0,0,6.99,17.,17.),StoppedScanState::Settling);
  EXPECT_EQ(consume(0,0,7.,17.,17.),StoppedScanState::Ready);
  EXPECT_EQ(confirmations,1u);
  EXPECT_EQ(consume(0,.07,7.1,17.1,17.1),StoppedScanState::Moving);
  EXPECT_EQ(confirmations,0u);EXPECT_DOUBLE_EQ(ros_start,17.1);
}
TEST(StoppedConfirmation,FrozenSnapshotRequiresCompleteStoppedUncanceledStage) {
  EXPECT_TRUE(stoppedConfirmationSnapshotReady(false,true,3,3,StoppedScanState::Ready));
  for(size_t count:{0u,1u,2u})EXPECT_FALSE(stoppedConfirmationSnapshotReady(false,true,count,3,StoppedScanState::Ready));
  EXPECT_FALSE(stoppedConfirmationSnapshotReady(true,true,3,3,StoppedScanState::Ready));
  EXPECT_FALSE(stoppedConfirmationSnapshotReady(false,false,3,3,StoppedScanState::Ready));
  EXPECT_FALSE(stoppedConfirmationSnapshotReady(false,true,3,3,StoppedScanState::Moving));
  EXPECT_FALSE(stoppedConfirmationSnapshotReady(false,true,3,3,StoppedScanState::Settling));
}
