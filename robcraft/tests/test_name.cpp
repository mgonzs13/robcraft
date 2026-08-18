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

#include <catch2/catch_test_macros.hpp>

#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/world/world.hpp"

using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;

TEST_CASE("Name component", "[name]") {
    robcraft::engine::world::World world;

    SECTION("add and retrieve name") {
        auto e = world.create_entity();
        world.add_component<robcraft::engine::ecs::Name>(
            e, robcraft::engine::ecs::Name{"test_entity"});

        auto* name = world.get_component<robcraft::engine::ecs::Name>(e);
        REQUIRE(name != nullptr);
        REQUIRE(name->value == "test_entity");
    }

    SECTION("default name is empty") {
        robcraft::engine::ecs::Name n;
        REQUIRE(n.value.empty());
    }
}
