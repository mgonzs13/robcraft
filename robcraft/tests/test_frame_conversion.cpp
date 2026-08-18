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

#include "robcraft/engine/math/frame_conversion.hpp"

using namespace robcraft::engine::math;
using Catch::Approx;

TEST_CASE("Sim position maps to REP-103 (X forward, Z up)", "[frame_conversion]") {
    // Sim is Y-up/Z-forward; REP-103 is X-forward/Z-up.
    robcraft::engine::math::Vec3 sim_pos(1.0, 2.0, 3.0);
    robcraft::engine::math::Vec3 rep = robcraft::engine::math::sim_to_rep103_position(sim_pos);
    REQUIRE(rep.x == Approx(3.0));  // sim forward (+Z) -> rep x
    REQUIRE(rep.y == Approx(1.0));  // sim left (+X) -> rep y
    REQUIRE(rep.z == Approx(2.0));  // sim up (+Y) -> rep z
}

TEST_CASE("Sim identity orientation stays identity in REP-103", "[frame_conversion]") {
    auto rep = robcraft::engine::math::sim_to_rep103_orientation(
        robcraft::engine::math::Quaternion::identity());
    REQUIRE(rep.w == Approx(1.0).margin(1e-9));
    REQUIRE(rep.x == Approx(0.0).margin(1e-9));
    REQUIRE(rep.y == Approx(0.0).margin(1e-9));
    REQUIRE(rep.z == Approx(0.0).margin(1e-9));
}

TEST_CASE("Sim heading about Y maps to yaw about Z in REP-103", "[frame_conversion]") {
    auto q_sim =
        robcraft::engine::math::Quaternion::from_euler(0.0, 1.5707963, 0.0);  // 90 deg about Y
    auto q_rep = robcraft::engine::math::sim_to_rep103_orientation(q_sim);
    auto e = q_rep.to_euler();
    REQUIRE(e.x == Approx(0.0).margin(0.001));
    REQUIRE(e.y == Approx(0.0).margin(0.001));
    REQUIRE(e.z == Approx(1.5707963).margin(0.001));
}

TEST_CASE("Sim heading and forward stay consistent after conversion", "[frame_conversion]") {
    // A robot turned 90 deg about sim-Y (left). Its sim forward (+Z) becomes
    // +X; REP-103 forward (+X) turned 90 deg about Z becomes +Y. Both must
    // agree after the frame change.
    robcraft::engine::math::Vec3 fwd_sim =
        robcraft::engine::math::Quaternion::from_euler(0.0, 1.5707963, 0.0)
            .rotate(robcraft::engine::math::Vec3(0, 0, 1));
    robcraft::engine::math::Vec3 fwd_rep =
        robcraft::engine::math::sim_to_rep103_orientation(
            robcraft::engine::math::Quaternion::from_euler(0.0, 1.5707963, 0.0))
            .rotate(robcraft::engine::math::Vec3(1, 0, 0));

    robcraft::engine::math::Vec3 fwd_sim_in_rep =
        robcraft::engine::math::sim_to_rep103_position(fwd_sim);
    REQUIRE(fwd_rep.x == Approx(fwd_sim_in_rep.x).margin(1e-9));
    REQUIRE(fwd_rep.y == Approx(fwd_sim_in_rep.y).margin(1e-9));
    REQUIRE(fwd_rep.z == Approx(fwd_sim_in_rep.z).margin(1e-9));
}
