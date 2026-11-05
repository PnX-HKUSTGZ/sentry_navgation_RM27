/**
 * @file execution.hpp
 * @brief 定义由主线程参与的同步分区执行接口。
 */
#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <type_traits>

namespace zlio {

/// 计算整数均匀分区的边界。
/// 初始化平面数和线程数受 int 接口约束，余数乘积在 64 位 size_t 中可表示。
inline std::size_t partitionBoundary(std::size_t count, std::size_t partitions, std::size_t index) {
    if (partitions == 0 || index > partitions)
        throw std::invalid_argument("invalid task partition bounds");
    return (count / partitions) * index + (count % partitions) * index / partitions;
}

/// 由单个调用线程拥有，管理固定辅助线程与同步计算批次。
class BatchExecutor {
public:
    /// 设置包含主线程在内的最大并行分区数。
    explicit BatchExecutor(std::size_t concurrency);
    /// 停止辅助线程并等待其退出。
    ~BatchExecutor();
    /// 禁止复制线程所有权。
    BatchExecutor(const BatchExecutor&) = delete;
    /// 禁止复制线程所有权。
    BatchExecutor& operator=(const BatchExecutor&) = delete;

    /// 串行提交编号连续的分区，并在全部完成后传播任务异常。
    template <class Function>
    void run(std::size_t partitions, Function&& function) {
        using Callable = std::remove_reference_t<Function>;
        execute(partitions, std::addressof(function), [](const void* context, std::size_t index) {
            auto* callable = const_cast<Callable*>(static_cast<const Callable*>(context));
            (*callable)(index);
        });
    }

private:
    using Task = void (*)(const void*, std::size_t);
    /// 发布任务描述并等待所有分区完成。
    void execute(std::size_t partitions, const void* context, Task task);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace zlio
