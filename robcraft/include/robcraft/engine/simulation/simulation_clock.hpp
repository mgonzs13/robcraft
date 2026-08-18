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

#include <cstdint>

namespace robcraft::engine::simulation {

using namespace robcraft::engine::simulation;

/** @brief Accumulates real time and converts it into fixed simulation ticks. */
class SimulationClock {
public:
    /**
     * @brief Constructs a clock with a fixed timestep.
     * @param dt Duration of one simulation tick in seconds.
     */
    explicit SimulationClock(double dt = 0.01);

    /**
     * @brief Advances the clock and returns the number of ticks to process.
     * @param real_dt Real elapsed time in seconds.
     * @return Number of full simulation ticks accumulated.
     */
    int32_t step(double real_dt);
    /**
     * @brief Adds real elapsed time to the accumulator.
     * @param real_dt Real elapsed time in seconds.
     */
    void update(double real_dt);
    /** @brief Number of full simulation ticks currently pending. */
    int32_t ticks_to_process() const;
    /**
     * @brief Consumes pending ticks, advancing simulation time.
     * @param count Number of ticks to consume.
     */
    void consume_ticks(int32_t count);

    /** @brief Duration of one simulation tick in seconds. */
    double dt() const { return this->dt_; }
    /** @brief Total simulated time in seconds. */
    double time() const { return this->time_; }
    /** @brief Resets time, tick count, and the accumulator. */
    void reset();

private:
    /** @brief Duration of one simulation tick in seconds. */
    double dt_;
    /** @brief Total simulated time in seconds. */
    double time_ = 0.0;
    /** @brief Total number of ticks simulated. */
    uint64_t tick_count_ = 0;
    /** @brief Unconsumed real time in seconds. */
    double accumulator_ = 0.0;
};

}  // namespace robcraft::engine::simulation
