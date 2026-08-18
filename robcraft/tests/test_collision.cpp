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

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/collision/collision.hpp"

using namespace robcraft::engine::collision;
using Catch::Approx;

TEST_CASE("AABB from box collider", "[collision]") {
    robcraft::engine::collision::BoxCollider box{robcraft::engine::math::Vec3(1.0, 0.5, 2.0)};
    robcraft::engine::math::Vec3 pos(3.0, 0.0, 1.0);

    auto aabb = robcraft::engine::collision::AABB::from_box(pos, box);

    REQUIRE(aabb.min.x == Approx(2.0));
    REQUIRE(aabb.min.y == Approx(-0.5));
    REQUIRE(aabb.min.z == Approx(-1.0));
    REQUIRE(aabb.max.x == Approx(4.0));
    REQUIRE(aabb.max.y == Approx(0.5));
    REQUIRE(aabb.max.z == Approx(3.0));
}

TEST_CASE("AABB overlap — separated", "[collision]") {
    auto a = robcraft::engine::collision::AABB{{0, 0, 0}, {1, 1, 1}};
    auto b = robcraft::engine::collision::AABB{{2, 0, 0}, {3, 1, 1}};

    REQUIRE(!a.overlaps(b));
}

TEST_CASE("AABB overlap — overlapping", "[collision]") {
    auto a = robcraft::engine::collision::AABB{{0, 0, 0}, {2, 2, 2}};
    auto b = robcraft::engine::collision::AABB{{1, 1, 1}, {3, 3, 3}};

    REQUIRE(a.overlaps(b));
}

TEST_CASE("AABB overlap — touching", "[collision]") {
    auto a = robcraft::engine::collision::AABB{{0, 0, 0}, {1, 1, 1}};
    auto b = robcraft::engine::collision::AABB{{1, 0, 0}, {2, 1, 1}};

    REQUIRE(a.overlaps(b));
}

TEST_CASE("AABB penetration", "[collision]") {
    auto a = robcraft::engine::collision::AABB{{0, 0, 0}, {2, 2, 2}};
    auto b = robcraft::engine::collision::AABB{{1.5, 0, 0}, {3.5, 2, 2}};

    auto pen = a.penetration(b);
    REQUIRE(pen.x == Approx(0.5));
    REQUIRE(pen.y == Approx(2.0));
    REQUIRE(pen.z == Approx(2.0));
}

TEST_CASE("AABB separation vector — X axis", "[collision]") {
    auto a = robcraft::engine::collision::AABB{{0, 0, 0}, {2, 2, 2}};
    auto b = robcraft::engine::collision::AABB{{1.8, 0, 0}, {3.8, 2, 2}};

    auto sep = a.separation_vector(b);
    REQUIRE(sep.x == Approx(-0.2));
    REQUIRE(sep.y == Approx(0.0));
    REQUIRE(sep.z == Approx(0.0));
}

TEST_CASE("AABB separation vector — Z axis", "[collision]") {
    auto a = robcraft::engine::collision::AABB{{0, 0, 0}, {2, 2, 2}};
    auto b = robcraft::engine::collision::AABB{{0, 0, 1.8}, {2, 2, 3.8}};

    auto sep = a.separation_vector(b);
    REQUIRE(sep.x == Approx(0.0));
    REQUIRE(sep.y == Approx(0.0));
    REQUIRE(sep.z == Approx(-0.2));
}

namespace {

/** @brief Overlapping AABBs for the resolution tests (B is to the right of A). */
struct OverlapPair {
    robcraft::engine::collision::AABB a;  // AABB around position (0,0,0), half extents (1,1,1)
    robcraft::engine::collision::AABB b;  // AABB around position (1.5,0,0), half extents (1,1,1)
    OverlapPair() {
        this->a = robcraft::engine::collision::AABB{{-1, -1, -1}, {1, 1, 1}};
        this->b = robcraft::engine::collision::AABB{{0.5, -1, -1}, {2.5, 1, 1}};
    }
};

}  // namespace

TEST_CASE("Overlap resolution — both stationary: nothing moves", "[collision]") {
    OverlapPair p;
    robcraft::engine::math::Vec3 pa(0.0, 0.0, 0.0);
    robcraft::engine::math::Vec3 pb(1.5, 0.0, 0.0);

    robcraft::engine::collision::resolve_overlap(p.a, p.b, robcraft::engine::math::Vec3(0, 0, 0),
                                                 robcraft::engine::math::Vec3(0, 0, 0), pa, pb);

    REQUIRE(pa.x == Approx(0.0));
    REQUIRE(pb.x == Approx(1.5));
}

TEST_CASE("Overlap resolution — mover vs stationary: only mover corrected", "[collision]") {
    OverlapPair p;
    robcraft::engine::math::Vec3 pa(0.0, 0.0, 0.0);
    robcraft::engine::math::Vec3 pb(1.5, 0.0, 0.0);

    // A drives +x into a parked B; B must stay put.
    robcraft::engine::collision::resolve_overlap(p.a, p.b, robcraft::engine::math::Vec3(1, 0, 0),
                                                 robcraft::engine::math::Vec3(0, 0, 0), pa, pb);

    REQUIRE(pa.x == Approx(-0.5));
    REQUIRE(pb.x == Approx(1.5));
}

TEST_CASE("Overlap resolution — stationary vs mover: only mover corrected", "[collision]") {
    OverlapPair p;
    robcraft::engine::math::Vec3 pa(0.0, 0.0, 0.0);
    robcraft::engine::math::Vec3 pb(1.5, 0.0, 0.0);

    // B drives -x into a parked A; A must stay put.
    robcraft::engine::collision::resolve_overlap(p.a, p.b, robcraft::engine::math::Vec3(0, 0, 0),
                                                 robcraft::engine::math::Vec3(-1, 0, 0), pa, pb);

    REQUIRE(pa.x == Approx(0.0));
    REQUIRE(pb.x == Approx(2.0));
}

TEST_CASE("Overlap resolution — head-on: correction split", "[collision]") {
    OverlapPair p;
    robcraft::engine::math::Vec3 pa(0.0, 0.0, 0.0);
    robcraft::engine::math::Vec3 pb(1.5, 0.0, 0.0);

    robcraft::engine::collision::resolve_overlap(p.a, p.b, robcraft::engine::math::Vec3(1, 0, 0),
                                                 robcraft::engine::math::Vec3(-1, 0, 0), pa, pb);

    // Each takes half of the 0.5 separation.
    REQUIRE(pa.x == Approx(-0.25));
    REQUIRE(pb.x == Approx(1.75));
}

TEST_CASE("Overlap resolution — mover driving away is not pushed back", "[collision]") {
    OverlapPair p;
    robcraft::engine::math::Vec3 pa(0.0, 0.0, 0.0);
    robcraft::engine::math::Vec3 pb(1.5, 0.0, 0.0);

    // A drives -x (away from B) while still overlapping; it must not be shoved into B.
    robcraft::engine::collision::resolve_overlap(p.a, p.b, robcraft::engine::math::Vec3(-1, 0, 0),
                                                 robcraft::engine::math::Vec3(0, 0, 0), pa, pb);

    REQUIRE(pa.x == Approx(0.0));
    REQUIRE(pb.x == Approx(1.5));
}

TEST_CASE("Rotated AABB", "[collision]") {
    SECTION("rotated box is larger") {
        robcraft::engine::collision::BoxCollider box{robcraft::engine::math::Vec3(1.0, 0.5, 1.0)};
        auto q = robcraft::engine::math::Quaternion::from_euler(0, 1.5708, 0);  // 90 degrees yaw
        auto aabb = robcraft::engine::collision::AABB::from_box(
            robcraft::engine::math::Vec3(0, 0, 0), box, q);
        // A 1x0.5x1 box rotated 90° about Y should have same extents (symmetric in x/z)
        REQUIRE(aabb.max.x == Approx(1.0));
        REQUIRE(aabb.max.z == Approx(1.0));
        REQUIRE(aabb.max.y == Approx(0.5));
    }

    SECTION("non-symmetric box rotated swaps x/z extents") {
        robcraft::engine::collision::BoxCollider box{robcraft::engine::math::Vec3(2.0, 0.5, 0.25)};
        auto q = robcraft::engine::math::Quaternion::from_euler(0, 1.5708, 0);  // 90 degrees yaw
        auto aabb = robcraft::engine::collision::AABB::from_box(
            robcraft::engine::math::Vec3(0, 0, 0), box, q);
        // Original: x extent 2, z extent 0.25. Rotated 90°: x extent 0.25, z extent 2
        REQUIRE(aabb.max.x == Approx(0.25).margin(0.01));
        REQUIRE(aabb.max.z == Approx(2.0).margin(0.01));
    }

    SECTION("unrotated matches original") {
        robcraft::engine::collision::BoxCollider box{robcraft::engine::math::Vec3(1.0, 2.0, 3.0)};
        auto aabb =
            robcraft::engine::collision::AABB::from_box(robcraft::engine::math::Vec3(1, 2, 3), box);
        auto aabb2 = robcraft::engine::collision::AABB::from_box(
            robcraft::engine::math::Vec3(1, 2, 3), box,
            robcraft::engine::math::Quaternion::identity());
        REQUIRE(aabb.min.x == Approx(aabb2.min.x));
        REQUIRE(aabb.max.x == Approx(aabb2.max.x));
        REQUIRE(aabb.max.z == Approx(aabb2.max.z));
    }
}
