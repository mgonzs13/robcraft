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

#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/world.hpp"

using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using Catch::Approx;

struct Mass {
    double value;
};

struct Health {
    int hp;
    int max_hp;
};

TEST_CASE("World creates and destroys entities", "[world]") {
    robcraft::engine::world::World world;

    auto e = world.create_entity();
    REQUIRE(world.valid(e));

    world.destroy_entity(e);
    REQUIRE(!world.valid(e));
}

TEST_CASE("World adds and retrieves components", "[world]") {
    robcraft::engine::world::World world;
    auto e = world.create_entity();

    robcraft::engine::ecs::Transform3D tf{robcraft::engine::math::Vec3(1.0, 2.0, 3.0)};
    world.add_component<robcraft::engine::ecs::Transform3D>(e, tf);

    REQUIRE(world.has_component<robcraft::engine::ecs::Transform3D>(e));

    auto* got = world.get_component<robcraft::engine::ecs::Transform3D>(e);
    REQUIRE(got != nullptr);
    REQUIRE(got->position.x == Approx(1.0));
    REQUIRE(got->position.y == Approx(2.0));
    REQUIRE(got->position.z == Approx(3.0));
}

TEST_CASE("World removes components", "[world]") {
    robcraft::engine::world::World world;
    auto e = world.create_entity();

    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});
    REQUIRE(world.has_component<robcraft::engine::ecs::Transform3D>(e));

    world.remove_component<robcraft::engine::ecs::Transform3D>(e);
    REQUIRE(!world.has_component<robcraft::engine::ecs::Transform3D>(e));
}

TEST_CASE("World handles multiple component types", "[world]") {
    robcraft::engine::world::World world;
    auto e = world.create_entity();

    world.add_component<robcraft::engine::ecs::Transform3D>(
        e, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(5.0, 0.0, 0.0)});
    world.add_component<Mass>(e, Mass{10.0});
    world.add_component<Health>(e, Health{100, 100});

    REQUIRE(world.has_component<robcraft::engine::ecs::Transform3D>(e));
    REQUIRE(world.has_component<Mass>(e));
    REQUIRE(world.has_component<Health>(e));

    REQUIRE(world.get_component<Mass>(e)->value == Approx(10.0));
    REQUIRE(world.get_component<Health>(e)->hp == 100);
}

TEST_CASE("World destroy removes all components", "[world]") {
    robcraft::engine::world::World world;
    auto e = world.create_entity();

    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<Mass>(e, Mass{5.0});

    world.destroy_entity(e);

    REQUIRE(!world.valid(e));
    REQUIRE(world.get_component<Mass>(e) == nullptr);
}

TEST_CASE("World clear resets lighting and sky to defaults", "[world]") {
    robcraft::engine::world::World world;
    robcraft::engine::lighting::SceneLighting l;
    l.sun_direction = robcraft::engine::math::Vec3(1.0, 0.0, 0.0);
    l.sun_color = robcraft::engine::math::Vec3(0.1f, 0.2f, 0.3f);
    l.ambient_color = robcraft::engine::math::Vec3(0.9f, 0.8f, 0.7f);
    world.set_lighting(l);
    robcraft::engine::lighting::Sky s;
    s.zenith_color = robcraft::engine::math::Vec3(0.1f, 0.2f, 0.3f);
    s.horizon_color = robcraft::engine::math::Vec3(0.4f, 0.5f, 0.6f);
    world.set_sky(s);

    world.clear();

    REQUIRE(world.lighting().sun_direction.x == Approx(0.5));
    REQUIRE(world.lighting().sun_color.x == Approx(1.0f));
    REQUIRE(world.lighting().ambient_color.x == Approx(0.4f));
    REQUIRE(world.sky().zenith_color.x == Approx(0.4f));
    REQUIRE(world.sky().horizon_color.x == Approx(0.7f));
}
