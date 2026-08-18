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

#include "robcraft/engine/math/cell_range.hpp"

using namespace robcraft::engine::math;

TEST_CASE("Wall run follows dominant axis", "[cellrange]") {
    SECTION("horizontal run") {
        auto r = robcraft::engine::math::wall_run_cells(2, 3, 7, 3);
        REQUIRE(r.x0 == 2);
        REQUIRE(r.x1 == 7);
        REQUIRE(r.z0 == 3);
        REQUIRE(r.z1 == 3);
    }

    SECTION("horizontal run backwards") {
        auto r = robcraft::engine::math::wall_run_cells(7, 3, 2, 3);
        REQUIRE(r.x0 == 2);
        REQUIRE(r.x1 == 7);
    }

    SECTION("vertical run") {
        auto r = robcraft::engine::math::wall_run_cells(4, 1, 4, 6);
        REQUIRE(r.x0 == 4);
        REQUIRE(r.x1 == 4);
        REQUIRE(r.z0 == 1);
        REQUIRE(r.z1 == 6);
    }

    SECTION("tie favors horizontal") {
        auto r = robcraft::engine::math::wall_run_cells(2, 2, 5, 5);
        REQUIRE(r.x0 == 2);
        REQUIRE(r.x1 == 5);
        REQUIRE(r.z0 == r.z1);
    }

    SECTION("single cell") {
        auto r = robcraft::engine::math::wall_run_cells(3, 3, 3, 3);
        REQUIRE(r.x0 == 3);
        REQUIRE(r.x1 == 3);
        REQUIRE(r.z0 == 3);
        REQUIRE(r.z1 == 3);
    }
}

TEST_CASE("Floor rect covers both corners", "[cellrange]") {
    auto r = robcraft::engine::math::floor_rect_cells(1, 2, 5, 6);
    REQUIRE(r.x0 == 1);
    REQUIRE(r.x1 == 5);
    REQUIRE(r.z0 == 2);
    REQUIRE(r.z1 == 6);

    auto b = robcraft::engine::math::floor_rect_cells(5, 6, 1, 2);
    REQUIRE(b.x0 == 1);
    REQUIRE(b.x1 == 5);
    REQUIRE(b.z0 == 2);
    REQUIRE(b.z1 == 6);
}

TEST_CASE("Cell range horizontal detection", "[cellrange]") {
    REQUIRE(robcraft::engine::math::cell_range_is_horizontal(
        robcraft::engine::math::wall_run_cells(1, 2, 6, 2)));
    REQUIRE(!robcraft::engine::math::cell_range_is_horizontal(
        robcraft::engine::math::wall_run_cells(1, 2, 1, 6)));
}

TEST_CASE("Cell ranges merge on same axis", "[cellrange]") {
    SECTION("horizontal abut") {
        REQUIRE(robcraft::engine::math::cell_ranges_merge(
            robcraft::engine::math::wall_run_cells(0, 3, 2, 3),
            robcraft::engine::math::wall_run_cells(3, 3, 5, 3)));
    }

    SECTION("horizontal overlap") {
        REQUIRE(robcraft::engine::math::cell_ranges_merge(
            robcraft::engine::math::wall_run_cells(0, 3, 4, 3),
            robcraft::engine::math::wall_run_cells(2, 3, 5, 3)));
    }

    SECTION("horizontal gap") {
        REQUIRE(!robcraft::engine::math::cell_ranges_merge(
            robcraft::engine::math::wall_run_cells(0, 3, 2, 3),
            robcraft::engine::math::wall_run_cells(4, 3, 6, 3)));
    }

    SECTION("different rows") {
        REQUIRE(!robcraft::engine::math::cell_ranges_merge(
            robcraft::engine::math::wall_run_cells(0, 3, 5, 3),
            robcraft::engine::math::wall_run_cells(0, 4, 5, 4)));
    }

    SECTION("vertical abut") {
        REQUIRE(robcraft::engine::math::cell_ranges_merge(
            robcraft::engine::math::wall_run_cells(2, 0, 2, 3),
            robcraft::engine::math::wall_run_cells(2, 4, 2, 6)));
    }

    SECTION("single cell merges both orientations") {
        REQUIRE(robcraft::engine::math::cell_ranges_merge(
            robcraft::engine::math::wall_run_cells(4, 2, 4, 2),
            robcraft::engine::math::wall_run_cells(2, 2, 4, 2)));
        REQUIRE(robcraft::engine::math::cell_ranges_merge(
            robcraft::engine::math::wall_run_cells(4, 2, 4, 2),
            robcraft::engine::math::wall_run_cells(4, 0, 4, 2)));
    }
}
