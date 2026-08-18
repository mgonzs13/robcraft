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

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <vector>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/scene_entities.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/world/world.hpp"

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::world;

TEST_CASE("collect_scene_entities includes collider entities", "[scene_entities]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity wall = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(wall,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::ecs::Name>(wall, robcraft::engine::ecs::Name{"wall_1"});
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall, robcraft::engine::collision::BoxCollider{});

    auto ents = robcraft::engine::ecs::collect_scene_entities(world);
    REQUIRE(std::find(ents.begin(), ents.end(), wall) != ents.end());
}

TEST_CASE("collect_scene_entities includes doodads without colliders", "[scene_entities]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity floor = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(floor,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::ecs::Name>(floor, robcraft::engine::ecs::Name{"floor_1"});

    auto ents = robcraft::engine::ecs::collect_scene_entities(world);
    REQUIRE(std::find(ents.begin(), ents.end(), floor) != ents.end());
}

TEST_CASE("collect_scene_entities excludes point lights", "[scene_entities]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity light = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(light,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::ecs::Name>(light, robcraft::engine::ecs::Name{"light_1"});
    world.add_component<robcraft::engine::lighting::PointLight>(
        light, robcraft::engine::lighting::PointLight{});

    auto ents = robcraft::engine::ecs::collect_scene_entities(world);
    REQUIRE(std::find(ents.begin(), ents.end(), light) == ents.end());
}

TEST_CASE("collect_scene_entities excludes nameless transforms", "[scene_entities]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity nameless = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(nameless,
                                                            robcraft::engine::ecs::Transform3D{});

    auto ents = robcraft::engine::ecs::collect_scene_entities(world);
    REQUIRE(std::find(ents.begin(), ents.end(), nameless) == ents.end());
}

TEST_CASE("collect_scene_entities includes each entity once", "[scene_entities]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity wall = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(wall,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::ecs::Name>(wall, robcraft::engine::ecs::Name{"wall_1"});
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall, robcraft::engine::collision::BoxCollider{});
    robcraft::engine::core::Entity floor = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(floor,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::ecs::Name>(floor, robcraft::engine::ecs::Name{"floor_1"});

    auto ents = robcraft::engine::ecs::collect_scene_entities(world);
    REQUIRE(std::count(ents.begin(), ents.end(), wall) == 1);
    REQUIRE(std::count(ents.begin(), ents.end(), floor) == 1);
}
