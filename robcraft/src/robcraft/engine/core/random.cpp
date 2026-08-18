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

#include "robcraft/engine/core/random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robcraft/engine/math/constants.hpp"

namespace robcraft::engine::core {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

Random::Random(uint64_t seed) : state_(seed) {}

uint64_t Random::next_u64() {
    this->state_ ^= this->state_ >> 12;
    this->state_ ^= this->state_ << 25;
    this->state_ ^= this->state_ >> 27;
    return this->state_ * 0x2545F4914F6CDD1DULL;
}

double Random::uniform() {
    return static_cast<double>(this->next_u64()) / static_cast<double>(UINT64_MAX);
}

double Random::uniform(double min, double max) {
    return min + (max - min) * this->uniform();
}

double Random::gaussian(double mean, double stddev) {
    double u1 = this->uniform();
    double u2 = this->uniform();
    if (u1 < 1e-12) u1 = 1e-12;
    double z = std::sqrt(-2.0 * std::log(u1)) * std::cos(robcraft::engine::math::kTwoPi * u2);
    return mean + z * stddev;
}

}  // namespace robcraft::engine::core
