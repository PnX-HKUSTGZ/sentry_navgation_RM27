#pragma once

#include <cstdint>

namespace minco_controller {

// Guarded by MincoMpcController::data_mtx_. Tokens are exact nanosecond values
// copied from Path.header.stamp and MpcPositionCommand.planning_stamp.
class TrajectorySessionGate {
public:
  enum class PlanUpdate { REJECTED, AUTHORIZED, REFRESHED, CLEARED };
  enum class NormalDisposition { ACCEPT, WAIT_FOR_PLAN, REJECT };

  void configure() {
    active_ = false;
    has_plan_ = false;
    waiting_for_plan_ = false;
    expected_token_ = 0U;
    authorized_token_ = 0U;
  }

  void activate() {
    active_ = true;
    has_plan_ = false;
    waiting_for_plan_ = false;
    expected_token_ = 0U;
    authorized_token_ = 0U;
  }

  void deactivate() {
    active_ = false;
    has_plan_ = false;
    waiting_for_plan_ = false;
    expected_token_ = 0U;
    authorized_token_ = 0U;
  }

  bool announceBlock(uint64_t token) {
    if (!active_ || token == 0U ||
        (expected_token_ != 0U && token < expected_token_)) {
      return false;
    }

    expected_token_ = token;
    waiting_for_plan_ = true;
    if (!has_plan_ || authorized_token_ != token) {
      has_plan_ = false;
      authorized_token_ = 0U;
    }
    return true;
  }

  PlanUpdate setPlan(uint64_t token, bool has_plan) {
    if (!active_) {
      return PlanUpdate::REJECTED;
    }
    if (!has_plan || token == 0U) {
      has_plan_ = false;
      waiting_for_plan_ = false;
      authorized_token_ = 0U;
      if (token > expected_token_) {
        expected_token_ = token;
      }
      return PlanUpdate::CLEARED;
    }
    if (expected_token_ != 0U && token < expected_token_) {
      return PlanUpdate::REJECTED;
    }
    if (has_plan_ && authorized_token_ == token) {
      waiting_for_plan_ = false;
      expected_token_ = token;
      return PlanUpdate::REFRESHED;
    }

    expected_token_ = token;
    authorized_token_ = token;
    has_plan_ = true;
    waiting_for_plan_ = false;
    return PlanUpdate::AUTHORIZED;
  }

  NormalDisposition classifyNormal(uint64_t token) const {
    if (!active_ || token == 0U) {
      return NormalDisposition::REJECT;
    }
    if (has_plan_ && token == authorized_token_) {
      return NormalDisposition::ACCEPT;
    }
    if (waiting_for_plan_ && token == expected_token_) {
      return NormalDisposition::WAIT_FOR_PLAN;
    }
    return NormalDisposition::REJECT;
  }

  bool active() const { return active_; }
  bool hasPlan() const { return has_plan_; }
  uint64_t expectedToken() const { return expected_token_; }
  uint64_t authorizedToken() const { return authorized_token_; }

private:
  bool active_{false};
  bool has_plan_{false};
  bool waiting_for_plan_{false};
  uint64_t expected_token_{0U};
  uint64_t authorized_token_{0U};
};

} // namespace minco_controller
