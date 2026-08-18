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
#include <limits>

#include "robcraft/engine/core/rate_gate.hpp"

using namespace robcraft::engine::core;
using Catch::Approx;

TEST_CASE("RateGate allows the first publish", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE(gate.due(0.0, 0.5));
}

TEST_CASE("RateGate blocks publishes before the interval elapses", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE(gate.due(0.0, 0.5));
    REQUIRE_FALSE(gate.due(0.2, 0.5));
    REQUIRE_FALSE(gate.due(0.499, 0.5));
}

TEST_CASE("RateGate allows a publish once the interval has elapsed", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE(gate.due(0.0, 0.5));
    REQUIRE_FALSE(gate.due(0.2, 0.5));
    REQUIRE(gate.due(0.5, 0.5));
}

TEST_CASE("RateGate reschedules from the last allowed time", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE(gate.due(0.0, 0.5));
    REQUIRE(gate.due(0.5, 0.5));
    REQUIRE_FALSE(gate.due(0.7, 0.5));
    REQUIRE(gate.due(1.0, 0.5));
}

TEST_CASE("RateGate applies a live interval change", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE(gate.due(0.0, 0.5));
    REQUIRE_FALSE(gate.due(0.4, 0.5));
    // A live rate change to 4 Hz (0.25 s interval) applies from the next
    // allowed publish: the old 0.5 s schedule stands, then the new interval
    // takes over for the following publish.
    REQUIRE(gate.due(0.5, 0.25));
    REQUIRE_FALSE(gate.due(0.7, 0.25));
    REQUIRE(gate.due(0.75, 0.25));
}

TEST_CASE("RateGate disables on non-positive intervals", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE_FALSE(gate.due(0.0, 0.0));
    REQUIRE_FALSE(gate.due(1.0, -0.5));
    REQUIRE_FALSE(gate.due(100.0, 0.0));
}

TEST_CASE("RateGate disables on infinite intervals", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE_FALSE(gate.due(0.0, std::numeric_limits<double>::infinity()));
    REQUIRE_FALSE(gate.due(1.0, std::numeric_limits<double>::infinity()));
}

TEST_CASE("RateGate does not flood after a stall", "[rate_gate]") {
    robcraft::engine::core::RateGate gate;
    REQUIRE(gate.due(0.0, 0.5));
    // Long stall to 10 s: the grid resumes; a couple of make-up passes may land
    // early, but the gate must not pass on every subsequent call (no flood).
    int passes = 0;
    for (double t = 10.0; t < 12.0; t += 0.05) {
        if (gate.due(t, 0.5)) ++passes;
    }
    // 2 s at 0.5 s intervals is ~4 passes; a make-up pass may add one.
    REQUIRE(passes >= 4);
    REQUIRE(passes <= 6);
}
