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
#include <cstddef>

#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec2.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/mesh.hpp"

using namespace robcraft::engine::math;
using namespace robcraft::renderer;

using Catch::Approx;

TEST_CASE("Vec2 basic operations", "[math]") {
    robcraft::engine::math::Vec2 a(1.0, 2.0);
    robcraft::engine::math::Vec2 b(3.0, 4.0);

    auto sum = a + b;
    REQUIRE(sum.x == Approx(4.0));
    REQUIRE(sum.y == Approx(6.0));

    auto diff = b - a;
    REQUIRE(diff.x == Approx(2.0));
    REQUIRE(diff.y == Approx(2.0));

    auto scaled = a * 3.0;
    REQUIRE(scaled.x == Approx(3.0));
    REQUIRE(scaled.y == Approx(6.0));

    REQUIRE(a.length() == Approx(std::sqrt(5.0)));
    REQUIRE(a.dot(b) == Approx(11.0));
}

TEST_CASE("Vec3 basic operations", "[math]") {
    robcraft::engine::math::Vec3 a(1.0, 2.0, 3.0);
    robcraft::engine::math::Vec3 b(4.0, 5.0, 6.0);

    REQUIRE((a + b).x == Approx(5.0));
    REQUIRE((a - b).y == Approx(-3.0));
    REQUIRE((a * 2.0).z == Approx(6.0));
    REQUIRE(a.dot(b) == Approx(32.0));

    auto cross = a.cross(b);
    REQUIRE(cross.x == Approx(-3.0));
    REQUIRE(cross.y == Approx(6.0));
    REQUIRE(cross.z == Approx(-3.0));
}

TEST_CASE("Quaternion identity does not rotate", "[math]") {
    auto q = robcraft::engine::math::Quaternion::identity();
    robcraft::engine::math::Vec3 v(1.0, 0.0, 0.0);
    auto rotated = q.rotate(v);
    REQUIRE(rotated.x == Approx(1.0));
    REQUIRE(rotated.y == Approx(0.0));
    REQUIRE(rotated.z == Approx(0.0));
}

TEST_CASE("Quaternion from axis angle rotates correctly", "[math]") {
    static const double pi = robcraft::engine::math::kPi;
    double half_pi = pi / 2.0;
    auto q = robcraft::engine::math::Quaternion::from_axis_angle(
        robcraft::engine::math::Vec3(0.0, 0.0, 1.0), half_pi);

    robcraft::engine::math::Vec3 v(1.0, 0.0, 0.0);
    auto rotated = q.rotate(v);
    REQUIRE(rotated.x == Approx(0.0).margin(1e-9));
    REQUIRE(rotated.y == Approx(1.0).margin(1e-9));
    REQUIRE(rotated.z == Approx(0.0).margin(1e-9));
}

TEST_CASE("Quaternion to_euler round trip", "[math]") {
    SECTION("identity is zero euler") {
        auto e = robcraft::engine::math::Quaternion::identity().to_euler();
        REQUIRE(e.x == Approx(0.0));
        REQUIRE(e.y == Approx(0.0));
        REQUIRE(e.z == Approx(0.0));
    }

    SECTION("yaw 90 degrees round trips") {
        auto q = robcraft::engine::math::Quaternion::from_euler(0.0, 0.0, 1.5707963);
        auto e = q.to_euler();
        REQUIRE(e.z == Approx(1.5707963).margin(0.001));
    }
}

TEST_CASE("Quaternion to_euler produces same rotation", "[math]") {
    SECTION("full round trip preserves rotation") {
        auto q = robcraft::engine::math::Quaternion::from_euler(0.1, 0.2, 0.3);
        auto e = q.to_euler();
        auto q2 = robcraft::engine::math::Quaternion::from_euler(e.x, e.y, e.z);
        // Check rotation equivalence: dot product should be ~±1 (same or negated quaternion)
        double dot = q.w * q2.w + q.x * q2.x + q.y * q2.y + q.z * q2.z;
        REQUIRE(std::abs(dot) > 0.99);
    }
}

TEST_CASE("Quaternion to_euler axis correctness", "[math]") {
    SECTION("yaw only (rotation about Z)") {
        auto q = robcraft::engine::math::Quaternion::from_euler(0.0, 0.0, 1.0);
        auto e = q.to_euler();
        REQUIRE(e.x == Approx(0.0).margin(0.001));
        REQUIRE(e.y == Approx(0.0).margin(0.001));
        REQUIRE(e.z == Approx(1.0).margin(0.001));
    }

    SECTION("pitch only (rotation about Y)") {
        auto q = robcraft::engine::math::Quaternion::from_euler(0.0, 0.5, 0.0);
        auto e = q.to_euler();
        REQUIRE(e.x == Approx(0.0).margin(0.001));
        REQUIRE(e.y == Approx(0.5).margin(0.001));
        REQUIRE(e.z == Approx(0.0).margin(0.001));
    }

    SECTION("roll only (rotation about X)") {
        auto q = robcraft::engine::math::Quaternion::from_euler(0.7, 0.0, 0.0);
        auto e = q.to_euler();
        REQUIRE(e.x == Approx(0.7).margin(0.001));
        REQUIRE(e.y == Approx(0.0).margin(0.001));
        REQUIRE(e.z == Approx(0.0).margin(0.001));
    }
}

TEST_CASE("Vertex layout size", "[math]") {
    REQUIRE(sizeof(robcraft::renderer::Vertex) == 14 * sizeof(float));
    REQUIRE(offsetof(robcraft::renderer::Vertex, u) == 9 * sizeof(float));
    REQUIRE(offsetof(robcraft::renderer::Vertex, tx) == 11 * sizeof(float));
}
