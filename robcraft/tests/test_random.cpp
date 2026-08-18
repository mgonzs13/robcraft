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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "robcraft/engine/core/random.hpp"

using namespace robcraft::engine::core;
using Catch::Approx;

TEST_CASE("Random deterministic with same seed", "[random]") {
    robcraft::engine::core::Random a(42);
    robcraft::engine::core::Random b(42);

    for (int i = 0; i < 100; ++i) {
        REQUIRE(a.next_u64() == b.next_u64());
    }
}

TEST_CASE("Random uniform in range", "[random]") {
    robcraft::engine::core::Random rng(7);
    for (int i = 0; i < 100; ++i) {
        double v = rng.uniform(5.0, 10.0);
        REQUIRE(v >= 5.0);
        REQUIRE(v <= 10.0);
    }
}

TEST_CASE("Random gaussian", "[random]") {
    robcraft::engine::core::Random rng(123);
    double sum = 0.0;
    int count = 10000;
    for (int i = 0; i < count; ++i) {
        sum += rng.gaussian(0.0, 1.0);
    }
    double mean = sum / count;
    REQUIRE(mean == Approx(0.0).margin(0.05));
}
