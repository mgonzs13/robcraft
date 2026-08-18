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

#include "robcraft/engine/core/logging.hpp"

using namespace robcraft::engine::core;

TEST_CASE("Logger creates and retrieves named logger", "[logging]") {
    spdlog::drop_all();

    auto logger = robcraft::engine::core::init_logger("test-log");
    REQUIRE(logger != nullptr);
    REQUIRE(logger->name() == "test-log");

    auto same = robcraft::engine::core::get_logger("test-log");
    REQUIRE(same == logger);
}
