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

#include "robcraft/engine/core/texture_size.hpp"

using namespace robcraft::engine::core;

TEST_CASE("texture_size_to_index maps known sizes", "[app_menu]") {
    REQUIRE(robcraft::engine::core::texture_size_to_index(256) == 0);
    REQUIRE(robcraft::engine::core::texture_size_to_index(512) == 1);
    REQUIRE(robcraft::engine::core::texture_size_to_index(1024) == 2);
}

TEST_CASE("texture_size_to_index falls back to 0 for unknown sizes", "[app_menu]") {
    REQUIRE(robcraft::engine::core::texture_size_to_index(0) == 0);
    REQUIRE(robcraft::engine::core::texture_size_to_index(128) == 0);
    REQUIRE(robcraft::engine::core::texture_size_to_index(2048) == 0);
}

TEST_CASE("index_to_texture_size maps indices", "[app_menu]") {
    REQUIRE(robcraft::engine::core::index_to_texture_size(0) == 256);
    REQUIRE(robcraft::engine::core::index_to_texture_size(1) == 512);
    REQUIRE(robcraft::engine::core::index_to_texture_size(2) == 1024);
}

TEST_CASE("index_to_texture_size clamps unknown indices", "[app_menu]") {
    REQUIRE(robcraft::engine::core::index_to_texture_size(-1) == 256);
    REQUIRE(robcraft::engine::core::index_to_texture_size(7) == 256);
}

TEST_CASE("size/index mapping round-trips", "[app_menu]") {
    for (int size : {256, 512, 1024}) {
        REQUIRE(robcraft::engine::core::index_to_texture_size(
                    robcraft::engine::core::texture_size_to_index(size)) == size);
    }
}
