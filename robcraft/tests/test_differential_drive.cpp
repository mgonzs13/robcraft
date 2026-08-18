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

#include "robcraft/robots/differential_drive/differential_drive.hpp"

using namespace robcraft::robots::differential_drive;

using Catch::Approx;

TEST_CASE("DifferentialDrive kinematics — straight line", "[differential_drive]") {
    robcraft::robots::differential_drive::DifferentialDrive dd;
    dd.wheel_base = 0.42;
    dd.left_velocity = 1.0;
    dd.right_velocity = 1.0;

    REQUIRE(dd.linear_velocity() == Approx(1.0));
    REQUIRE(dd.angular_velocity() == Approx(0.0));
}

TEST_CASE("DifferentialDrive kinematics — point turn", "[differential_drive]") {
    robcraft::robots::differential_drive::DifferentialDrive dd;
    dd.wheel_base = 0.42;
    dd.left_velocity = -1.0;
    dd.right_velocity = 1.0;

    REQUIRE(dd.linear_velocity() == Approx(0.0));
    REQUIRE(dd.angular_velocity() == Approx(2.0 / 0.42).margin(0.01));
}

TEST_CASE("DifferentialDrive kinematics — forward right", "[differential_drive]") {
    robcraft::robots::differential_drive::DifferentialDrive dd;
    dd.wheel_base = 0.5;
    dd.left_velocity = 0.5;
    dd.right_velocity = 1.0;

    REQUIRE(dd.linear_velocity() == Approx(0.75));
    REQUIRE(dd.angular_velocity() == Approx(1.0));
}

TEST_CASE("DifferentialDrive defaults", "[differential_drive]") {
    robcraft::robots::differential_drive::DifferentialDrive dd;
    REQUIRE(dd.wheel_base == Approx(0.42));
    REQUIRE(dd.left_velocity == Approx(0.0));
    REQUIRE(dd.right_velocity == Approx(0.0));
}

TEST_CASE("DifferentialDrive default odom rate", "[differential_drive]") {
    robcraft::robots::differential_drive::DifferentialDrive drive;
    REQUIRE(drive.odom_rate == Approx(30.0));
    REQUIRE(drive.odom_noise_stddev == Approx(0.0));
}
