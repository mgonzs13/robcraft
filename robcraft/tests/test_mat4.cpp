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

#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/quaternion.hpp"

using namespace robcraft::engine::math;
using Catch::Approx;

TEST_CASE("Mat4 identity multiplication", "[math]") {
    robcraft::engine::math::Mat4 a;
    robcraft::engine::math::Mat4 b;
    auto c = a * b;

    for (int i = 0; i < 16; ++i) {
        if (i == 0 || i == 5 || i == 10 || i == 15)
            REQUIRE(c.data[i] == Approx(1.0f));
        else
            REQUIRE(c.data[i] == Approx(0.0f));
    }
}

TEST_CASE("Mat4 perspective projection", "[math]") {
    float fov = static_cast<float>(robcraft::engine::math::deg_to_rad(90.0));
    auto proj = robcraft::engine::math::Mat4::perspective(fov, 1.0f, 0.1f, 100.0f);

    REQUIRE(proj.data[15] == Approx(0.0f));

    REQUIRE(proj.data[0] == Approx(1.0f).margin(0.01f));
    REQUIRE(proj.data[5] == Approx(1.0f).margin(0.01f));
}

TEST_CASE("Mat4 look at", "[math]") {
    robcraft::engine::math::Vec3 eye(0.0, 0.0, 5.0);
    robcraft::engine::math::Vec3 center(0.0, 0.0, 0.0);
    robcraft::engine::math::Vec3 up(0.0, 1.0, 0.0);

    auto view = robcraft::engine::math::Mat4::look_at(eye, center, up);

    REQUIRE(view.data[0] == Approx(1.0f));
    REQUIRE(view.data[15] == Approx(1.0f));
}

TEST_CASE("Mat4 from position and rotation", "[math]") {
    robcraft::engine::math::Vec3 pos(3.0, 0.0, 0.0);
    auto rot = robcraft::engine::math::Quaternion::identity();
    auto model = robcraft::engine::math::Mat4::from_position_rotation(pos, rot);

    REQUIRE(model.data[12] == Approx(3.0f));
    REQUIRE(model.data[13] == Approx(0.0f));
    REQUIRE(model.data[14] == Approx(0.0f));
    REQUIRE(model.data[15] == Approx(1.0f));
}

TEST_CASE("Mat4 orthographic maps planes correctly", "[math]") {
    auto m = robcraft::engine::math::Mat4::orthographic(-10, 10, -10, 10, 0.1f, 200.0f);
    REQUIRE(m.data[0] == Approx(0.1f));
    REQUIRE(m.data[5] == Approx(0.1f));
    REQUIRE(m.data[10] == Approx(-2.0f / 199.9f));
    REQUIRE(m.data[12] == Approx(0.0f));
    REQUIRE(m.data[13] == Approx(0.0f));
    REQUIRE(m.data[15] == Approx(1.0f));
}

TEST_CASE("Mat4 inverse round-trips to identity", "[math]") {
    robcraft::engine::math::Mat4 m = robcraft::engine::math::Mat4::from_position_rotation(
        robcraft::engine::math::Vec3(3.0, 2.0, 1.0),
        robcraft::engine::math::Quaternion::from_euler(0.5, 0.2, 0.3));
    auto inv = m.inverse();
    auto id = m * inv;

    for (int i = 0; i < 16; ++i) {
        if (i == 0 || i == 5 || i == 10 || i == 15)
            REQUIRE(id.data[i] == Approx(1.0f).margin(1e-4f));
        else
            REQUIRE(id.data[i] == Approx(0.0f).margin(1e-4f));
    }
}

TEST_CASE("Mat4 inverse of a projection unprojects NDC", "[math]") {
    robcraft::engine::math::Mat4 proj = robcraft::engine::math::Mat4::perspective(
        static_cast<float>(robcraft::engine::math::deg_to_rad(60.0)), 1.0f, 0.1f, 100.0f);
    robcraft::engine::math::Mat4 inv = proj.inverse();

    // inv * proj must be the identity (authoritative inverse check).
    robcraft::engine::math::Mat4 roundtrip = proj * inv;
    REQUIRE(roundtrip.data[0] == Approx(1.0f).margin(1e-4f));
    REQUIRE(roundtrip.data[5] == Approx(1.0f).margin(1e-4f));
    REQUIRE(roundtrip.data[10] == Approx(1.0f).margin(1e-4f));
    REQUIRE(roundtrip.data[15] == Approx(1.0f).margin(1e-4f));
    REQUIRE(roundtrip.data[1] == Approx(0.0f).margin(1e-4f));
    REQUIRE(roundtrip.data[3] == Approx(0.0f).margin(1e-4f));
}

TEST_CASE("Mat4 inverse of a singular matrix returns identity", "[math]") {
    robcraft::engine::math::Mat4 m;
    m.data.fill(0.0f);
    auto inv = m.inverse();
    REQUIRE(inv.data[0] == Approx(1.0f));
    REQUIRE(inv.data[15] == Approx(1.0f));
}
