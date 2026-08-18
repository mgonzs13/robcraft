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

#include "robcraft/engine/world/gravity_step.hpp"
#include "robcraft/engine/world/world.hpp"

using namespace robcraft::engine::world;

using Catch::Approx;

TEST_CASE("apply_gravity pins a resting robot to the ground", "[gravity]") {
    double y = 0.5;
    double vy = 0.0;
    apply_gravity(y, vy, 0.5, 9.81, 0.01);
    REQUIRE(y == Approx(0.5));
    REQUIRE(vy == Approx(0.0));
}

TEST_CASE("apply_gravity accelerates an airborne robot downward", "[gravity]") {
    double y = 5.5;
    double vy = 0.0;
    apply_gravity(y, vy, 0.5, 9.81, 0.01);
    REQUIRE(vy == Approx(-9.81 * 0.01));
    REQUIRE(y == Approx(5.5 - 9.81 * 0.01 * 0.01));
}

TEST_CASE("apply_gravity lands a robot that crosses the ground in one tick", "[gravity]") {
    double y = 0.501;  // 1 mm above ground; semi-implicit step falls g*dt^2 ~ 24.5 mm
    double vy = 0.0;
    apply_gravity(y, vy, 0.5, 9.81, 0.05);
    REQUIRE(y == Approx(0.5));
    REQUIRE(vy == Approx(0.0));
}

TEST_CASE("apply_gravity with upward velocity leaves the ground", "[gravity]") {
    double y = 0.5;
    double vy = 1.0;
    apply_gravity(y, vy, 0.5, 9.81, 0.01);
    REQUIRE(y == Approx(0.5 + (1.0 - 9.81 * 0.01) * 0.01));
    REQUIRE(vy == Approx(1.0 - 9.81 * 0.01));
}

TEST_CASE("apply_gravity with zero gravity keeps an airborne robot in place", "[gravity]") {
    double y = 2.0;
    double vy = 0.0;
    apply_gravity(y, vy, 0.5, 0.0, 0.01);
    REQUIRE(y == Approx(2.0));
    REQUIRE(vy == Approx(0.0));
}

TEST_CASE("World defaults to Earth gravity", "[gravity]") {
    robcraft::engine::world::World world;
    REQUIRE(world.gravity() == Approx(9.81));
}
