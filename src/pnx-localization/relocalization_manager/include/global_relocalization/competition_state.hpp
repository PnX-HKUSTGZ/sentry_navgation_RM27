// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstdint>

namespace global_relocalization {
// Session contract, owned by the action-state mutex. A start event is terminal
// for preparation: even an early event can never resume pre-match spinning.
class CompetitionState {
public:
  enum class Start { Inactive, Duplicate, TooEarly, Accepted };
  uint64_t begin() {++generation_;active_=true;prepared_=started_=release_=false;return generation_;}
  bool prepared(uint64_t generation) {
    if(!active_||started_||generation!=generation_)return false;
    prepared_=true;return true;
  }
  Start start() {
    if(!active_)return Start::Inactive;
    if(started_)return Start::Duplicate;
    started_=true;
    return prepared_?Start::Accepted:Start::TooEarly;
  }
  bool finish(bool success) {
    release_=active_&&success&&prepared_&&started_;active_=false;return release_;
  }
  bool active()const{return active_;}
  bool ready()const{return active_&&prepared_&&!started_;}
  bool isPrepared()const{return prepared_;}
  bool started()const{return started_;}
  bool releaseControl()const{return release_;}
  uint64_t generation()const{return generation_;}
private:
  uint64_t generation_{0};bool active_{false},prepared_{false},started_{false},release_{false};
};
}
