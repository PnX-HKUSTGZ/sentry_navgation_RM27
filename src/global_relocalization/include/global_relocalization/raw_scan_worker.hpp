// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <cmath>
#include <algorithm>
#include <optional>

namespace global_relocalization {
// One ordered processor. Input, in-flight and unconsumed output share the same
// 32-slot/128-MiB bound: a stalled consumer cannot grow a second result FIFO.
template<class Payload, class Result> class RawScanWorker {
public:
  using Clock=std::chrono::steady_clock;
  static constexpr size_t capacity=32, max_message_bytes=4*1024*1024;
  struct Item {Payload payload;uint64_t generation;size_t bytes;double stamp,receipt_ros;Clock::time_point received;};
  struct Completed {Item item;Result result;double queue_wait_sec,worker_wall_sec,source_age_sec;};
  struct Stats {
    uint64_t generation{0},received_count{0},overflow_count{0},oversized_count{0},stale_count{0},ignored_count{0},canceled_count{0};
    size_t queue_depth{0},result_depth{0},outstanding{0},bytes{0},high_watermark{0},max_bytes{0};
    double latest_stamp{-1},receipt_wall_sec{0},receipt_ros_sec{0},receipt_delay_sec{0},max_receipt_delay_sec{0},max_stamp_gap{0},max_callback_gap{0};
    std::string error;
  };
  using Processor=std::function<Result(const Item&)>;
  RawScanWorker(Processor processor,std::function<double()> ros_now,double timeout=1.5):processor_(std::move(processor)),ros_now_(std::move(ros_now)),timeout_(timeout),thread_([this]{run();}){}
  ~RawScanWorker(){close();}
  RawScanWorker(const RawScanWorker&)=delete;
  uint64_t begin() {
    std::unique_lock<std::mutex> lock(mutex_);invalidate();cv_.wait(lock,[this]{return !busy_;});
    const auto generation=stats_.generation;stats_=Stats{};stats_.generation=generation;active_=capture_=true;watermark_=-1;return generation;
  }
  void cancel(){std::lock_guard<std::mutex> lock(mutex_);invalidate();cv_.notify_all();}
  void waitIdle(){std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return !busy_;});}
  void freeze() {
    std::unique_lock<std::mutex> lock(mutex_);capture_=false;
    stats_.ignored_count+=queue_.size();for(const auto & item:queue_)release(item);queue_.clear();
    cv_.wait(lock,[this]{return !busy_;});
  }
  void watermark(double stamp){std::lock_guard<std::mutex> lock(mutex_);watermark_=stamp;cv_.notify_one();}
  bool enqueue(Payload payload,size_t bytes,double stamp,double receipt_ros,Clock::time_point received=Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);if(!active_)return true;
    ++stats_.received_count;
    const double wall=std::chrono::duration<double>(received.time_since_epoch()).count();
    if(stats_.latest_stamp>=0){stats_.max_stamp_gap=std::max(stats_.max_stamp_gap,stamp-stats_.latest_stamp);stats_.max_callback_gap=std::max(stats_.max_callback_gap,wall-stats_.receipt_wall_sec);}
    const double previous=stats_.latest_stamp;stats_.latest_stamp=stamp;stats_.receipt_wall_sec=wall;stats_.receipt_ros_sec=receipt_ros;
    stats_.receipt_delay_sec=receipt_ros-stamp;stats_.max_receipt_delay_sec=std::max(stats_.max_receipt_delay_sec,stats_.receipt_delay_sec);
    if(!stats_.error.empty())return false;
    if(!std::isfinite(stamp)||!std::isfinite(receipt_ros)||(previous>=0&&stamp<=previous))return fault("raw scan timestamp not strictly increasing");
    if(previous>=0&&stamp-previous>.35+1e-9)return fault("raw source scan coverage gap");
    if(bytes>max_message_bytes){++stats_.oversized_count;return fault("raw scan exceeds 4 MiB message limit");}
    if(receipt_ros-stamp>=timeout_||receipt_ros-stamp<-.1){++stats_.stale_count;return fault("raw source scan timestamp is stale or in the future");}
    if(!capture_){++stats_.ignored_count;return true;}
    if(stats_.outstanding>=capacity){++stats_.overflow_count;return fault("raw scan FIFO overflow (32 outstanding frames)");}
    queue_.push_back({std::move(payload),stats_.generation,bytes,stamp,receipt_ros,received});++stats_.outstanding;stats_.bytes+=bytes;
    stats_.high_watermark=std::max(stats_.high_watermark,stats_.outstanding);stats_.max_bytes=std::max(stats_.max_bytes,stats_.bytes);cv_.notify_one();return true;
  }
  std::deque<Completed> takeResults(){std::lock_guard<std::mutex> lock(mutex_);std::deque<Completed> out;out.swap(results_);for(const auto & done:out)release(done.item);return out;}
  std::optional<Completed> takeResult(){std::lock_guard<std::mutex> lock(mutex_);if(results_.empty())return {};auto done=std::move(results_.front());results_.pop_front();release(done.item);return done;}
  Stats stats()const{std::lock_guard<std::mutex> lock(mutex_);auto out=stats_;out.queue_depth=queue_.size();out.result_depth=results_.size();return out;}
  void close(){ {std::lock_guard<std::mutex> lock(mutex_);if(stopping_)return;invalidate();stopping_=true;cv_.notify_all();}if(thread_.joinable())thread_.join();}
private:
  bool fault(const std::string & error){stats_.error=error;capture_=false;cv_.notify_all();return false;}
  void release(const Item & item){--stats_.outstanding;stats_.bytes-=item.bytes;}
  void invalidate(){active_=capture_=false;++stats_.generation;stats_.canceled_count+=queue_.size()+results_.size();for(const auto & item:queue_)release(item);for(const auto & done:results_)release(done.item);queue_.clear();results_.clear();}
  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while(!stopping_) {
      cv_.wait_for(lock,std::chrono::milliseconds(10),[this]{return stopping_||(active_&&capture_&&!queue_.empty()&&queue_.front().stamp<=watermark_);});if(stopping_)break;
      if(!active_||!capture_||queue_.empty())continue;
      if(queue_.front().stamp>watermark_){const double age=ros_now_()-queue_.front().stamp,wait=std::chrono::duration<double>(Clock::now()-queue_.front().received).count();if(age>=timeout_||wait>=timeout_){++stats_.stale_count;fault("raw scan expired waiting for IMU coverage");}continue;}
      Item item=std::move(queue_.front());queue_.pop_front();busy_=true;lock.unlock();
      const auto started=Clock::now();const double wait=std::chrono::duration<double>(started-item.received).count(),age=ros_now_()-item.stamp;
      Result result{};std::string error;
      if(wait>=timeout_||age>=timeout_||age<-.1)error="raw scan expired before worker processing";
      else try{result=processor_(item);}catch(const std::exception & e){error=std::string("raw scan worker exception: ")+e.what();}catch(...){error="raw scan worker unknown exception";}
      const double duration=std::chrono::duration<double>(Clock::now()-started).count();lock.lock();busy_=false;
      if(item.generation!=stats_.generation||!active_){++stats_.canceled_count;release(item);}
      else if(!error.empty()){if(error.find("expired")!=std::string::npos)++stats_.stale_count;fault(error);release(item);}
      else {item.payload=Payload{};results_.push_back({std::move(item),std::move(result),wait,duration,age});}
      cv_.notify_all();
    }
  }
  Processor processor_;std::function<double()> ros_now_;double timeout_;
  mutable std::mutex mutex_;std::condition_variable cv_;std::deque<Item> queue_;std::deque<Completed> results_;Stats stats_;
  bool active_{false},capture_{false},busy_{false},stopping_{false};double watermark_{-1};std::thread thread_;
};
}
