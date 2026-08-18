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
#include <cstdio>

#include "robcraft/engine/lighting/sky.hpp"
#include "robcraft/engine/world/world.hpp"

using namespace robcraft::engine::lighting;
using namespace robcraft::engine::world;
using Catch::Approx;

TEST_CASE("Sky defaults", "[sky]") {
    robcraft::engine::lighting::Sky sky;
    REQUIRE(sky.zenith_color.x == Approx(0.4f));
    REQUIRE(sky.zenith_color.y == Approx(0.5f));
    REQUIRE(sky.zenith_color.z == Approx(0.7f));
    REQUIRE(sky.horizon_color.x == Approx(0.7f));
    REQUIRE(sky.horizon_color.y == Approx(0.8f));
    REQUIRE(sky.horizon_color.z == Approx(0.9f));
}

TEST_CASE("World stores sky", "[sky]") {
    robcraft::engine::world::World world;
    REQUIRE(world.sky().horizon_color.z == Approx(0.9f));
    robcraft::engine::lighting::Sky s;
    s.zenith_color = robcraft::engine::math::Vec3(0.1f, 0.2f, 0.3f);
    world.set_sky(s);
    REQUIRE(world.sky().zenith_color.x == Approx(0.1f));
    REQUIRE(world.sky().zenith_color.y == Approx(0.2f));
}
