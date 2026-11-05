#include "rm27_backend/cloud_store.hpp"
#include <iomanip>
#include <pcl/io/pcd_io.h>

namespace rm27_backend {
namespace {
/// 按点数组已分配容量计算缓存字节数，避免低估预留内存。
std::size_t bytes(const Cloud &c) {
  return c.points.capacity() * sizeof(pcl::PointXYZI);
}
/// 向 CSV 追加平移和四元数，四元数顺序为 x、y、z、w。
void pose(std::ostream &o, const Pose &p) {
  Eigen::Quaterniond q(p.linear());
  o << ',' << p.translation().x() << ',' << p.translation().y() << ','
    << p.translation().z() << ',' << q.x() << ',' << q.y() << ',' << q.z()
    << ',' << q.w();
}
} // namespace
/// 校验资源预算；开启归档时拒绝复用已有目录，建立清单和索引后启动工作线程。
CloudStore::CloudStore(StorageConfig c) : config_(std::move(c)) {
  if (config_.cache_bytes == 0 || config_.queue_size == 0 ||
      config_.disk_bytes == 0)
    throw std::invalid_argument("invalid storage bounds");
  if (!config_.directory.empty()) {
    if (std::filesystem::exists(config_.directory))
      throw std::invalid_argument(
          "archive directory already exists; use a new run directory");
    std::filesystem::create_directories(
        std::filesystem::path(config_.directory) / "clouds");
    std::ofstream manifest(std::filesystem::path(config_.directory) /
                           "manifest.txt");
    manifest << "rm27_backend_archive 1\ncloud_frame=keyframe IMU body\n"
                "world_frame=camera_init\ncovariance=local-right "
                "translation,rotation\n"
                "estimates=at_creation,not_final_optimized\nresume_supported="
                "false\n";
    if (!manifest)
      throw std::runtime_error("cannot write archive manifest");
    std::ofstream index(std::filesystem::path(config_.directory) /
                        "keyframes.csv");
    index << "id,segment,instance,epoch,sequence,stamp_ns,tx,ty,tz,qx,qy,qz,qw,"
             "ex,ey,ez,eqx,eqy,eqz,eqw,degraded\n";
    std::ofstream edges(std::filesystem::path(config_.directory) /
                        "odometry.csv");
    edges << "from,to,dt,distance,angle,tx,ty,tz,qx,qy,qz,qw,var_x,var_y,var_z,"
             "var_rx,var_ry,var_rz\n";
    if (!index || !edges)
      throw std::runtime_error("cannot write archive index");
    thread_ = std::thread(&CloudStore::worker, this);
  }
}
/// 唤醒工作线程处理剩余任务，等待线程退出后释放存储资源。
CloudStore::~CloudStore() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable())
    thread_.join();
}
/// 将局部点云放入按插入顺序逐出的有界缓存，并按需提交归档任务。
/// 持锁期间只操作内存；归档队列满时记录丢弃，不阻塞图更新等待磁盘。
void CloudStore::put(const Keyframe &k, Cloud::ConstPtr c,
                     std::optional<OdometryMeasurement> edge) {
  if (!c || c->empty())
    throw std::invalid_argument("empty keyframe cloud");
  std::lock_guard<std::mutex> lock(mutex_);
  if (cache_.count(k.id))
    throw std::invalid_argument("duplicate cloud id");
  const auto size = bytes(*c);
  if (size <= config_.cache_bytes) {
    while (!order_.empty() && stats_.cache_bytes + size > config_.cache_bytes) {
      const auto key = order_.front();
      order_.pop_front();
      stats_.cache_bytes -= bytes(*cache_.at(key));
      cache_.erase(key);
      ++stats_.evictions;
    }
    cache_.emplace(k.id, c);
    order_.push_back(k.id);
    stats_.cache_bytes += size;
  } else {
    ++stats_.evictions;
  }
  if (!config_.directory.empty()) {
    if (queue_.size() >= config_.queue_size) {
      ++stats_.dropped;
      stats_.last_error = "archive queue full";
    } else {
      queue_.push_back({k, c, std::move(edge)});
      wake_.notify_one();
    }
  }
}
/// 优先返回缓存中的只读点云；未命中时在锁外读取对应归档文件。
/// 无归档或文件尚不存在返回空指针，已有文件读取失败则抛异常。
Cloud::ConstPtr CloudStore::get(Key k) const {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cache_.find(k);
    if (it != cache_.end())
      return it->second;
  }
  if (config_.directory.empty())
    return {};
  const auto file = std::filesystem::path(config_.directory) / "clouds" /
                    (std::to_string(k) + ".pcd");
  if (!std::filesystem::exists(file))
    return {};
  Cloud::Ptr c(new Cloud);
  if (pcl::io::loadPCDFile(file.string(), *c) < 0)
    throw std::runtime_error("archive cloud read failed");
  return c;
}
/// 在锁内复制资源与错误统计，队列数包含正在执行的任务。
StorageStats CloudStore::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto s = stats_;
  s.cache_clouds = cache_.size();
  s.queued = queue_.size() + (working_ ? 1 : 0);
  return s;
}
/// 等待队列为空且没有正在执行的写盘任务，不保证此前任务均成功。
void CloudStore::flush() {
  std::unique_lock<std::mutex> lock(mutex_);
  // 同时检查等待队列和正在执行项，避免尚未写完就结束等待。
  idle_.wait(lock, [&] { return queue_.empty() && !working_; });
}
/// 工作线程循环取出任务，在锁外写盘，并将异常转为存储错误统计。
/// 退出请求到达后继续处理已排队任务，最后通知等待 flush 的调用方。
void CloudStore::worker() {
  for (;;) {
    Work work;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // 有新任务或收到退出请求时唤醒；退出前仍会处理已有队列。
      wake_.wait(lock, [&] { return stop_ || !queue_.empty(); });
      if (queue_.empty() && stop_)
        return;
      work = std::move(queue_.front());
      queue_.pop_front();
      working_ = true;
    }
    try {
      save(work);
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.errors;
      stats_.last_error = e.what();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      working_ = false;
    }
    idle_.notify_all();
  }
}
/// 检查保守磁盘配额，先将 PCD 写到临时文件再改名，随后追加关键帧与里程计索引。
/// 跨文件写入不是原子事务，失败可能留下未索引点云；成功后才累计保存数量和配额。
void CloudStore::save(const Work &w) {
  // Conservative reservation covers PCD payload, headers and metadata. Stop
  // recording at the quota; never silently delete previous archive entries.
  const std::uint64_t reserve = w.cloud->size() * sizeof(pcl::PointXYZI) + 8192;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stats_.disk_bytes + reserve > config_.disk_bytes)
      throw std::runtime_error("archive disk quota exceeded");
  }
  const auto base = std::filesystem::path(config_.directory);
  const auto final = base / "clouds" / (std::to_string(w.keyframe.id) + ".pcd");
  const auto temp = final.string() + ".tmp";
  if (pcl::io::savePCDFileBinary(temp, *w.cloud) < 0)
    throw std::runtime_error("PCD write failed");
  std::filesystem::rename(temp, final);
  const auto &k = w.keyframe;
  std::ofstream index(base / "keyframes.csv", std::ios::app);
  index << std::setprecision(17);
  index << k.id << ',' << k.segment << ',' << k.frame.identity.instance << ','
        << k.frame.identity.epoch << ',' << k.frame.sequence << ','
        << k.frame.stamp_ns;
  pose(index, k.frame.odom);
  pose(index, k.estimate);
  index << ',' << k.frame.degraded << '\n';
  index.flush();
  if (!index)
    throw std::runtime_error("keyframe index write failed");
  if (w.edge) {
    const auto &e = *w.edge;
    std::ofstream f(base / "odometry.csv", std::ios::app);
    f << std::setprecision(17);
    f << e.from << ',' << e.to << ',' << e.seconds << ',' << e.distance << ','
      << e.angle;
    pose(f, e.relative);
    for (int i = 0; i < 6; ++i)
      f << ',' << e.covariance(i, i);
    f << '\n';
    f.flush();
    if (!f)
      throw std::runtime_error("odometry index write failed");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.disk_bytes += reserve;
  ++stats_.saved;
}
} // namespace rm27_backend
