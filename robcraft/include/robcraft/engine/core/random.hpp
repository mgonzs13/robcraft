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

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/** @brief xorshift-style deterministic PRNG. */
class Random {
public:
    /**
     * @brief Constructs a generator with the given seed.
     * @param seed Initial seed, default 12345.
     */
    explicit Random(uint64_t seed = 12345);

    /**
     * @brief Returns the next 64-bit random value.
     * @return A pseudo-random 64-bit unsigned integer.
     */
    uint64_t next_u64();

    /**
     * @brief Returns a uniform value in [0, 1].
     * @return A pseudo-random double in [0, 1].
     */
    double uniform();

    /**
     * @brief Returns a uniform value in [min, max].
     * @param min Lower bound.
     * @param max Upper bound.
     * @return A pseudo-random double in [min, max].
     */
    double uniform(double min, double max);

    /**
     * @brief Returns a normally distributed value.
     * @param mean The distribution mean.
     * @param stddev The standard deviation.
     * @return A pseudo-random double from the distribution.
     */
    double gaussian(double mean, double stddev);

private:
    /** @brief Current generator state. */
    uint64_t state_;
};

}  // namespace robcraft::engine::core
