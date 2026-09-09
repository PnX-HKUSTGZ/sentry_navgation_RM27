#pragma once

#include <cstddef>
#include <mutex>
#include <optional>
#include <utility>

namespace rog_map::detail
{

template <typename ValueT>
class LatestValueMailbox
{
public:
  struct Delivery
  {
    ValueT value;
    std::size_t pending_count{0U};
  };

  void store(ValueT value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(value);
    ++pending_count_;
  }

  std::optional<Delivery> takeLatest()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_count_ == 0U) {
      return std::nullopt;
    }

    Delivery delivery{std::move(*latest_), pending_count_};
    latest_.reset();
    pending_count_ = 0U;
    return delivery;
  }

private:
  std::mutex mutex_;
  std::optional<ValueT> latest_;
  std::size_t pending_count_{0U};
};

}  // namespace rog_map::detail
