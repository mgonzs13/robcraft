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

#pragma once

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/** @brief Allows at most one gate pass per configured interval. */
class RateGate {
public:
    /** @brief Returns true when `now` permits a pass and schedules the next one.
     *  @param now Current wall-clock time in seconds.
     *  @param interval Minimum seconds between consecutive passes. Non-positive
     *         or non-finite intervals disable the gate (always returns false).
     *  @return True when a pass is allowed at `now`; false when not due or when
     *          `interval` is invalid. */
    bool due(double now, double interval);

private:
    /** @brief Earliest wall time (seconds) the next pass may happen. */
    double next_ = 0.0;
};

}  // namespace robcraft::engine::core
