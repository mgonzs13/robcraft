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

#include "robcraft/engine/core/name_utils.hpp"

using namespace robcraft::engine::core;

TEST_CASE("sanitize_ros_name produces valid namespace parts", "[name_utils]") {
    REQUIRE(robcraft::engine::core::sanitize_ros_name("Robot Mike") == "robot_mike");
    REQUIRE(robcraft::engine::core::sanitize_ros_name("robot__mike") == "robot_mike");
    REQUIRE(robcraft::engine::core::sanitize_ros_name("_foo_") == "foo");
    REQUIRE(robcraft::engine::core::sanitize_ros_name("123") == "_123");
    REQUIRE(robcraft::engine::core::sanitize_ros_name("!!!") == "");
}

TEST_CASE("robot_base_name strips entity-id suffix and falls back", "[name_utils]") {
    REQUIRE(robcraft::engine::core::robot_base_name("robot_mike_5", 5) == "robot_mike");
    REQUIRE(robcraft::engine::core::robot_base_name("robot_mike_5", 7) == "robot_mike_5");
    REQUIRE(robcraft::engine::core::robot_base_name("robot_mike", 5) == "robot_mike");
    REQUIRE(robcraft::engine::core::robot_base_name("", 5) == "robot");
}

TEST_CASE("robot_namespace appends a per-type index", "[name_utils]") {
    REQUIRE(robcraft::engine::core::robot_namespace("robot_mike_5", 5, 2) == "robot_mike_2");
    REQUIRE(robcraft::engine::core::robot_namespace("robot_leela", 6, 1) == "robot_leela_1");
    REQUIRE(robcraft::engine::core::robot_namespace("", 5, 1) == "robot_1");
}
