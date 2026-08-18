// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robcraft/engine/core/rate_gate.hpp"

#include <algorithm>
#include <cmath>

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

bool RateGate::due(double now, double interval) {
    // Invalid intervals (rate <= 0 or rate == 0 → +inf) disable the gate:
    // never publish, preventing an infinite loop-rate flood from malformed rates.
    if (interval <= 0.0 || !std::isfinite(interval)) return false;
    // Grid-based schedule: advance by the fixed interval so the average publish
    // rate tracks the target even against a coarse loop. Clamp so a long stall
    // can never lag the grid far enough to cause a catch-up burst.
    this->next_ = std::max(this->next_, now - interval);
    if (now < this->next_) return false;
    this->next_ += interval;
    return true;
}

}  // namespace robcraft::engine::core
