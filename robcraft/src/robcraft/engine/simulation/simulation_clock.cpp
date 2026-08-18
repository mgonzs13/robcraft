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

#include "robcraft/engine/simulation/simulation_clock.hpp"

namespace robcraft::engine::simulation {

using namespace robcraft::engine::simulation;

SimulationClock::SimulationClock(double dt) : dt_(dt), time_(0.0), tick_count_(0) {}

void SimulationClock::update(double real_dt) {
    this->accumulator_ += real_dt;
}

int32_t SimulationClock::ticks_to_process() const {
    if (this->dt_ <= 0.0) return 0;
    return static_cast<int32_t>(this->accumulator_ / this->dt_);
}

void SimulationClock::consume_ticks(int32_t count) {
    this->accumulator_ -= count * this->dt_;
    this->tick_count_ += count;
    this->time_ += count * this->dt_;
}

int32_t SimulationClock::step(double real_dt) {
    this->update(real_dt);
    int32_t ticks = this->ticks_to_process();
    this->consume_ticks(ticks);
    return ticks;
}

void SimulationClock::reset() {
    this->time_ = 0.0;
    this->tick_count_ = 0;
    this->accumulator_ = 0.0;
}

}  // namespace robcraft::engine::simulation
