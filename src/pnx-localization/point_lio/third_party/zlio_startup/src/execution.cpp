/**
 * @file execution.cpp
 * @brief 执行同步计算批次并管理辅助线程的生命周期。
 */
#include "execution.hpp"

#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace zlio {

struct BatchExecutor::Impl {
    /// 初始化固定分区的异常存储。
    explicit Impl(std::size_t count) : concurrency(count), errors(count) {
        if (count == 0)
            throw std::invalid_argument("batch executor concurrency must be positive");
        threads.reserve(count - 1);
    }

    /// 结束所有辅助线程。
    ~Impl() { stop(); }

    /// 创建固定辅助线程，并清理创建失败前已取得的资源。
    void start() {
        if (!threads.empty())
            return;
        try {
            for (std::size_t index = 1; index < concurrency; ++index)
                threads.emplace_back([this, index] { work(index); });
        } catch (...) {
            stop();
            throw;
        }
    }

    /// 执行一个分区并保存其异常。
    void invoke(std::size_t index) noexcept {
        try {
            task(context, index);
        } catch (...) {
            errors[index] = std::current_exception();
        }
    }

    /// 等待新批次并执行本线程对应的分区。
    void work(std::size_t index) {
        std::size_t seen = 0;
        std::unique_lock<std::mutex> lock(mutex);
        while (true) {
            ready.wait(lock, [&] { return stopped || generation != seen; });
            if (stopped)
                return;
            seen = generation;
            if (index >= active)
                continue;
            lock.unlock();
            invoke(index);
            lock.lock();
            --pending;
            completed.notify_one();
        }
    }

    /// 停止接收批次并回收线程。
    void stop() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stopped = true;
        }
        ready.notify_all();
        for (auto& thread : threads)
            if (thread.joinable())
                thread.join();
    }

    /// 同步发布任务，按分区编号传播首个异常。
    void execute(std::size_t partitions, const void* task_context, Task operation) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (stopped)
                throw std::runtime_error("batch executor has stopped and must be recreated");
            if (partitions == 0 || partitions > concurrency)
                throw std::invalid_argument(
                    "batch partition count must be positive and not exceed concurrency");
        }
        if (partitions > 1)
            start();
        {
            std::lock_guard<std::mutex> lock(mutex);
            active = partitions;
            pending = partitions - 1;
            context = task_context;
            task = operation;
            for (auto& error : errors)
                error = nullptr;
            ++generation;
        }
        ready.notify_all();
        invoke(0);
        {
            std::unique_lock<std::mutex> lock(mutex);
            completed.wait(lock, [&] { return pending == 0; });
            context = nullptr;
            task = nullptr;
        }
        for (std::size_t index = 0; index < partitions; ++index)
            if (errors[index])
                std::rethrow_exception(errors[index]);
    }

    const std::size_t concurrency;
    std::vector<std::exception_ptr> errors;
    std::vector<std::thread> threads;
    std::mutex mutex;
    std::condition_variable ready, completed;
    const void* context = nullptr;
    Task task = nullptr;
    bool stopped = false;
    std::size_t generation = 0, active = 0, pending = 0;
};

/// 创建由主线程和固定辅助线程共同执行的批执行器。
BatchExecutor::BatchExecutor(std::size_t concurrency) : impl_(std::make_unique<Impl>(concurrency)) {
}

/// 释放执行器及其线程资源。
BatchExecutor::~BatchExecutor() = default;

/// 将同步调用交给拥有线程资源的实现对象。
void BatchExecutor::execute(std::size_t partitions, const void* context, Task task) {
    impl_->execute(partitions, context, task);
}

}  // namespace zlio
