#pragma once

#include <algorithm>
#include <chrono>
#include <thread>

namespace minco_planner
{

// Use wall time: startup must still time out if a simulation clock is paused.
template<typename Predicate>
bool waitForStaticMap(Predicate ready, std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    if (ready()) {
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::min(
        deadline - now, std::chrono::steady_clock::duration(std::chrono::milliseconds(20))));
  } while (true);
}

}  // namespace minco_planner
