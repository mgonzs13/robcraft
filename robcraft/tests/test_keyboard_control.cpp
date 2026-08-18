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

#include "robcraft/engine/core/keyboard_control.hpp"

using namespace robcraft::engine::core;

using Catch::Approx;

TEST_CASE("KeyboardDriveControl no keys does not command", "[keyboard_control]") {
    KeyboardDriveControl control;
    auto cmd = control.update(false, false, false, false, false);
    REQUIRE_FALSE(cmd.active);
    REQUIRE(cmd.linear == Approx(0.0));
    REQUIRE(cmd.angular == Approx(0.0));
}

TEST_CASE("KeyboardDriveControl forward while I held", "[keyboard_control]") {
    KeyboardDriveControl control;
    auto cmd = control.update(true, false, false, false, false);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(1.0));
    REQUIRE(cmd.angular == Approx(0.0));
}

TEST_CASE("KeyboardDriveControl backward while K held", "[keyboard_control]") {
    KeyboardDriveControl control;
    auto cmd = control.update(false, true, false, false, false);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(-1.0));
    REQUIRE(cmd.angular == Approx(0.0));
}

TEST_CASE("KeyboardDriveControl turn left while J held", "[keyboard_control]") {
    KeyboardDriveControl control;
    auto cmd = control.update(false, false, true, false, false);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(0.0));
    REQUIRE(cmd.angular == Approx(1.0));
}

TEST_CASE("KeyboardDriveControl turn right while L held", "[keyboard_control]") {
    KeyboardDriveControl control;
    auto cmd = control.update(false, false, false, true, false);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(0.0));
    REQUIRE(cmd.angular == Approx(-1.0));
}

TEST_CASE("KeyboardDriveControl forward plus turn combines", "[keyboard_control]") {
    KeyboardDriveControl control;
    auto cmd = control.update(true, false, true, false, false);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(1.0));
    REQUIRE(cmd.angular == Approx(1.0));
}

TEST_CASE("KeyboardDriveControl stop key commands zero", "[keyboard_control]") {
    KeyboardDriveControl control;
    auto cmd = control.update(false, false, false, false, true);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(0.0));
    REQUIRE(cmd.angular == Approx(0.0));
}

TEST_CASE("KeyboardDriveControl releasing movement key stops once", "[keyboard_control]") {
    KeyboardDriveControl control;
    control.update(true, false, false, false, false);
    auto stop = control.update(false, false, false, false, false);
    REQUIRE(stop.active);
    REQUIRE(stop.linear == Approx(0.0));
    REQUIRE(stop.angular == Approx(0.0));
}

TEST_CASE("KeyboardDriveControl staying idle after release does not command",
          "[keyboard_control]") {
    KeyboardDriveControl control;
    control.update(true, false, false, false, false);
    control.update(false, false, false, false, false);
    auto cmd = control.update(false, false, false, false, false);
    REQUIRE_FALSE(cmd.active);
}

TEST_CASE("KeyboardDriveControl re-holding after release moves again", "[keyboard_control]") {
    KeyboardDriveControl control;
    control.update(true, false, false, false, false);
    control.update(false, false, false, false, false);
    auto cmd = control.update(true, false, false, false, false);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(1.0));
}

TEST_CASE("KeyboardDriveControl holding through frames stays active", "[keyboard_control]") {
    KeyboardDriveControl control;
    control.update(true, false, false, false, false);
    auto cmd = control.update(true, false, false, false, false);
    REQUIRE(cmd.active);
    REQUIRE(cmd.linear == Approx(1.0));
}
