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

#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/collision/raycast.hpp"

using namespace robcraft::engine::collision;
using Catch::Approx;

TEST_CASE("Ray hits AABB from outside", "[raycast]") {
    robcraft::engine::collision::AABB box{{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    robcraft::engine::collision::Ray ray{{-1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}};

    auto hit = robcraft::engine::collision::ray_aabb_intersection(ray, box);
    REQUIRE(hit.has_value());
    REQUIRE(hit.value() == Approx(1.0));
}

TEST_CASE("Ray misses AABB", "[raycast]") {
    robcraft::engine::collision::AABB box{{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    robcraft::engine::collision::Ray ray{{-1.0, 1.0, 1.0}, {0.0, 1.0, 0.0}};

    auto hit = robcraft::engine::collision::ray_aabb_intersection(ray, box);
    REQUIRE(!hit.has_value());
}

TEST_CASE("Ray originates inside AABB", "[raycast]") {
    robcraft::engine::collision::AABB box{{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    robcraft::engine::collision::Ray ray{{1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}};

    auto hit = robcraft::engine::collision::ray_aabb_intersection(ray, box);
    REQUIRE(hit.has_value());
    REQUIRE(hit.value() == Approx(0.0));
}

TEST_CASE("Ray hits AABB diagonally", "[raycast]") {
    robcraft::engine::collision::AABB box{{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    robcraft::engine::collision::Ray ray{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};

    auto dir = ray.direction.normalized();
    ray.direction = dir;

    auto hit = robcraft::engine::collision::ray_aabb_intersection(ray, box);
    REQUIRE(hit.has_value());
}

TEST_CASE("Ray hits AABB at glancing angle", "[raycast]") {
    robcraft::engine::collision::AABB box{{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    robcraft::engine::collision::Ray ray{{-1.0, 0.5, 0.5}, {1.0, 0.0, 0.0}};

    auto hit = robcraft::engine::collision::ray_aabb_intersection(ray, box);
    REQUIRE(hit.has_value());
    REQUIRE(hit.value() == Approx(1.0));
}
