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

#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/world/world.hpp"

using namespace robcraft::engine::lighting;
using namespace robcraft::engine::world;
using Catch::Approx;

TEST_CASE("SceneLighting defaults match current hardcoded values", "[lighting]") {
    robcraft::engine::lighting::SceneLighting l;
    REQUIRE(l.sun_direction.x == Approx(0.5));
    REQUIRE(l.sun_direction.y == Approx(1.0));
    REQUIRE(l.sun_direction.z == Approx(0.3));
    REQUIRE(l.sun_intensity == Approx(1.0f));
    REQUIRE(l.ambient_intensity == Approx(1.0f));
    REQUIRE(l.shadows_enabled);
}

TEST_CASE("PointLight defaults", "[lighting]") {
    robcraft::engine::lighting::PointLight p;
    REQUIRE(p.color.x == Approx(1.0f));
    REQUIRE(p.intensity == Approx(1.0f));
    REQUIRE(p.range == Approx(8.0f));
}

TEST_CASE("World stores scene lighting", "[lighting]") {
    robcraft::engine::world::World world;
    REQUIRE(world.lighting().sun_intensity == Approx(1.0f));
    robcraft::engine::lighting::SceneLighting l;
    l.sun_direction = robcraft::engine::math::Vec3(0.2, 1.0, 0.1);
    l.sun_intensity = 2.5f;
    world.set_lighting(l);
    REQUIRE(world.lighting().sun_direction.x == Approx(0.2));
    REQUIRE(world.lighting().sun_intensity == Approx(2.5f));
}
