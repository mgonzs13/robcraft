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

#include "robcraft/engine/pathfinding/astar.hpp"
#include "robcraft/engine/world/terrain.hpp"

using namespace robcraft::engine::pathfinding;
using namespace robcraft::engine::world;

TEST_CASE("A* finds straight path", "[astar]") {
    robcraft::engine::world::Terrain t(10, 10, 1.0);
    double half = 5.0;
    auto start = robcraft::engine::math::Vec3(-3.5, 0, 0);
    auto end = robcraft::engine::math::Vec3(3.5, 0, 0);
    auto path = robcraft::engine::find_path(t, start, end);
    REQUIRE(path.size() > 1);
}

TEST_CASE("A* avoids blocked cells", "[astar]") {
    robcraft::engine::world::Terrain t(8, 8, 1.0);
    double half = 4.0;
    t.set_walkable(3, 3, false);
    t.set_walkable(3, 4, false);
    t.set_walkable(4, 3, false);
    t.set_walkable(4, 4, false);

    auto start = robcraft::engine::math::Vec3(-3.5, 0, -3.5);
    auto end = robcraft::engine::math::Vec3(3.5, 0, 3.5);
    auto path = robcraft::engine::find_path(t, start, end);
    REQUIRE(path.size() > 1);
    for (auto& p : path) {
        int cx, cz;
        t.world_to_cell(p.x, p.z, cx, cz);
        REQUIRE(t.is_walkable(cx, cz));
    }
}

TEST_CASE("A* returns empty for unreachable", "[astar]") {
    robcraft::engine::world::Terrain t(5, 5, 1.0);
    double half = 2.5;
    for (int x = 0; x < 5; ++x) t.set_walkable(x, 2, false);

    auto start = robcraft::engine::math::Vec3(-2.0, 0, -2.0);
    auto end = robcraft::engine::math::Vec3(-2.0, 0, 2.0);
    auto path = robcraft::engine::find_path(t, start, end);
    REQUIRE(path.empty());
}
