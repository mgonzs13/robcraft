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

#include "robcraft/renderer/reflection.hpp"

using namespace robcraft::renderer;

using Catch::Approx;

namespace {

/** @brief Transforms a world point by a column-major Mat4. */
robcraft::engine::math::Vec3 transform_point(const robcraft::engine::math::Mat4& m,
                                             const robcraft::engine::math::Vec3& p) {
    return robcraft::engine::math::Vec3(
        m.data[0] * p.x + m.data[4] * p.y + m.data[8] * p.z + m.data[12],
        m.data[1] * p.x + m.data[5] * p.y + m.data[9] * p.z + m.data[13],
        m.data[2] * p.x + m.data[6] * p.y + m.data[10] * p.z + m.data[14]);
}

}  // namespace

TEST_CASE("reflected_view renders the virtual image of world points", "[reflection]") {
    robcraft::engine::math::Mat4 view = robcraft::engine::math::Mat4::look_at(
        robcraft::engine::math::Vec3(0.0, 4.0, 8.0), robcraft::engine::math::Vec3(0.0, 0.0, 0.0),
        robcraft::engine::math::Vec3(0.0, 1.0, 0.0));
    robcraft::engine::math::Mat4 rv = robcraft::renderer::reflected_view(view, 2.0f);
    // A mirror shows the virtual image: the reflected view of p must equal the
    // original view of the mirrored point pm = (px, 2*plane_y - py, pz).
    const robcraft::engine::math::Vec3 pts[] = {
        robcraft::engine::math::Vec3(3.0, 6.0, -2.0), robcraft::engine::math::Vec3(-4.0, 1.0, 5.0),
        robcraft::engine::math::Vec3(2.0, -3.0, 1.0), robcraft::engine::math::Vec3(0.0, 0.0, 0.0)};
    for (const auto& p : pts) {
        robcraft::engine::math::Vec3 pm(p.x, 2.0f * 2.0f - p.y, p.z);
        robcraft::engine::math::Vec3 a = transform_point(rv, p);
        robcraft::engine::math::Vec3 b = transform_point(view, pm);
        REQUIRE(a.x == Approx(b.x));
        REQUIRE(a.y == Approx(b.y));
        REQUIRE(a.z == Approx(b.z));
    }
}

TEST_CASE("reflection_clip_plane keeps the upper half-space", "[reflection]") {
    auto p = robcraft::renderer::reflection_clip_plane(2.0f, 0.1f);
    REQUIRE(p[0] == Approx(0.0f));
    REQUIRE(p[1] == Approx(1.0f));
    REQUIRE(p[2] == Approx(0.0f));
    REQUIRE(p[3] == Approx(-1.9f));
    // Plane equation ax + by + cz + d: keep points with value >= 0.
    REQUIRE(p[1] * 2.0f + p[3] >= 0.0f);  // y = 2.0 (above clip threshold 1.9)
    REQUIRE(p[1] * 1.8f + p[3] < 0.0f);   // y = 1.8 (below)
}
