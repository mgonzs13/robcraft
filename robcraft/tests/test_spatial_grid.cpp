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
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "robcraft/engine/collision/spatial_grid.hpp"

using namespace robcraft::engine::collision;
using Catch::Approx;

TEST_CASE("SpatialGrid insert and query ray", "[spatial_grid]") {
    robcraft::engine::collision::SpatialGrid grid(1.0, -5.0, -5.0, 5.0, 5.0);

    robcraft::engine::collision::BoxCollider col{robcraft::engine::math::Vec3(0.5, 0.5, 0.5)};
    robcraft::engine::math::Vec3 pos(2.0, 0.0, 0.0);
    grid.insert(10, pos, col);

    robcraft::engine::collision::Ray ray{{-3.0, 0.5, 0.0}, {1.0, 0.0, 0.0}};
    auto candidates = grid.query_ray(ray, 10.0);

    REQUIRE(!candidates.empty());
    REQUIRE(candidates[0] == 10);
}

TEST_CASE("SpatialGrid ray misses all", "[spatial_grid]") {
    robcraft::engine::collision::SpatialGrid grid(1.0, -5.0, -5.0, 5.0, 5.0);

    robcraft::engine::collision::BoxCollider col{robcraft::engine::math::Vec3(0.5, 0.5, 0.5)};
    robcraft::engine::math::Vec3 pos(2.0, 0.0, 0.0);
    grid.insert(10, pos, col);

    robcraft::engine::collision::Ray ray{{-3.0, 0.5, 3.0}, {1.0, 0.0, 0.0}};
    auto candidates = grid.query_ray(ray, 10.0);

    REQUIRE(candidates.empty());
}

TEST_CASE("SpatialGrid multiple entities in same cell", "[spatial_grid]") {
    robcraft::engine::collision::SpatialGrid grid(2.0, -4.0, -4.0, 4.0, 4.0);

    robcraft::engine::collision::BoxCollider col{robcraft::engine::math::Vec3(0.5, 0.5, 0.5)};
    grid.insert(1, robcraft::engine::math::Vec3(0.5, 0.0, 0.5), col);
    grid.insert(2, robcraft::engine::math::Vec3(0.8, 0.0, 0.3), col);

    robcraft::engine::collision::Ray ray{{-2.0, 0.5, 0.5}, {1.0, 0.0, 0.0}};
    auto candidates = grid.query_ray(ray, 10.0);

    REQUIRE(candidates.size() >= 1);
}

TEST_CASE("SpatialGrid query AABB returns overlapping entities", "[spatial_grid]") {
    robcraft::engine::collision::SpatialGrid grid(2.0, -4.0, -4.0, 4.0, 4.0);

    robcraft::engine::collision::BoxCollider col{robcraft::engine::math::Vec3(0.5, 0.5, 0.5)};
    grid.insert(1, robcraft::engine::math::Vec3(1.0, 0.0, 1.0), col);
    grid.insert(2, robcraft::engine::math::Vec3(-1.0, 0.0, -1.0), col);
    grid.insert(3, robcraft::engine::math::Vec3(3.0, 0.0, 3.0), col);

    robcraft::engine::collision::AABB box{{-2.0, 0.0, -2.0}, {2.0, 0.0, 2.0}};
    auto hits = grid.query_aabb(box);

    REQUIRE(hits.size() == 2);
    REQUIRE(std::find(hits.begin(), hits.end(), 1) != hits.end());
    REQUIRE(std::find(hits.begin(), hits.end(), 2) != hits.end());
}

TEST_CASE("SpatialGrid query AABB deduplicates entities", "[spatial_grid]") {
    robcraft::engine::collision::SpatialGrid grid(1.0, -5.0, -5.0, 5.0, 5.0);

    robcraft::engine::collision::BoxCollider col{robcraft::engine::math::Vec3(1.5, 0.5, 1.5)};
    grid.insert(7, robcraft::engine::math::Vec3(0.0, 0.0, 0.0), col);

    robcraft::engine::collision::AABB box{{-1.0, 0.0, -1.0}, {1.0, 0.0, 1.0}};
    auto hits = grid.query_aabb(box);
    REQUIRE(hits.size() == 1);
}

TEST_CASE("SpatialGrid insert with rotation covers the rotated footprint", "[spatial_grid]") {
    robcraft::engine::collision::SpatialGrid grid(2.0, -4.0, -4.0, 4.0, 4.0);

    robcraft::engine::collision::BoxCollider col{robcraft::engine::math::Vec3(0.5, 0.5, 0.5)};
    robcraft::engine::math::Quaternion rot =
        robcraft::engine::math::Quaternion::from_euler(0.0, 0.7853981633974483, 0.0);
    grid.insert(5, robcraft::engine::math::Vec3(1.4, 0.0, 1.4), col, rot);

    // Rotated corner reaches x=2.107 (cell 3); the unrotated AABB stops at 1.9 (cell 2).
    robcraft::engine::collision::AABB query{{2.05, 0.0, 2.05}, {2.15, 0.0, 2.15}};
    auto hits = grid.query_aabb(query);
    REQUIRE(hits.size() == 1);
    REQUIRE(hits[0] == 5);
}
