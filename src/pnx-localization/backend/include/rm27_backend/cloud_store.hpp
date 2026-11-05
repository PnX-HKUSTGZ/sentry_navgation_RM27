#pragma once
#include "rm27_backend/types.hpp"
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <thread>

namespace rm27_backend {
/// 点云存储预算：缓存字节数、异步队列长度、保守磁盘配额和归档目录。
struct StorageConfig {
  std::size_t cache_bytes = 32 * 1024 * 1024, queue_size = 4;
  std::uint64_t disk_bytes = 1024ULL * 1024 * 1024;
  std::string
      directory; // 空值表示仅使用有界内存；归档不代表可恢复优化会话
};
/// 存储统计快照；缓存容量、队列积压、逐出、保存和错误信息分别计数。
struct StorageStats {
  std::size_t cache_bytes = 0, cache_clouds = 0, queued = 0;
  std::uint64_t evictions = 0, saved = 0, dropped = 0, errors = 0,
                disk_bytes = 0;
  std::string last_error;
};
/// 提供有界点云缓存与可选异步归档；调用方须保持共享点云只读。
/// 缓存和写盘队列分别限额，归档保存创建时状态，不恢复完整 iSAM2 会话。
class CloudStore {
public:
  /// 校验存储配置；启用归档时创建全新目录、索引并启动写盘线程。
  explicit CloudStore(StorageConfig config);
  /// 通知写盘线程退出，并等待已排队任务处理结束。
  ~CloudStore();
  /// 缓存关键帧局部点云并按需排队归档；超出预算时逐出旧缓存或记录归档丢弃。
  void put(const Keyframe &keyframe, Cloud::ConstPtr cloud,
           std::optional<OdometryMeasurement> edge);
  /// 优先读取内存云，否则从归档加载；缺失返回空指针，读盘失败抛异常。
  Cloud::ConstPtr get(Key key) const;
  /// 在锁内取得统计快照，queued 包含正在写入的任务。
  StorageStats stats() const;
  /// 等待当前队列和正在执行的归档任务处理完毕；成功与否需另查统计。
  void flush();
  /// 返回本次记录目录，空值表示未启用归档。
  const std::string &directory() const { return config_.directory; }

private:
  /// 异步写盘任务：关键帧快照、共享只读点云及可选里程计边。
  struct Work {
    Keyframe keyframe;
    Cloud::ConstPtr cloud;
    std::optional<OdometryMeasurement> edge;
  };
  /// 依次处理写盘任务，在锁外执行磁盘操作并记录异常。
  void worker();
  /// 写入一项点云和索引；先检查配额，失败由工作线程记录。
  void save(const Work &work);
  StorageConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable wake_, idle_;
  std::map<Key, Cloud::ConstPtr> cache_;
  std::deque<Key> order_;
  std::deque<Work> queue_;
  StorageStats stats_;
  bool stop_ = false, working_ = false;
  std::thread thread_;
};
} // 命名空间 rm27_backend
