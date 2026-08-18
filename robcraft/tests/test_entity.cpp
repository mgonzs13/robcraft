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

#include "robcraft/engine/core/entity.hpp"

using namespace robcraft::engine::core;

TEST_CASE("EntityManager creates valid entities", "[entity]") {
    robcraft::engine::core::EntityManager mgr;

    auto e1 = mgr.create();
    auto e2 = mgr.create();
    auto e3 = mgr.create();

    REQUIRE(e1 != robcraft::engine::core::INVALID_ENTITY);
    REQUIRE(e2 != robcraft::engine::core::INVALID_ENTITY);
    REQUIRE(e3 != robcraft::engine::core::INVALID_ENTITY);
    REQUIRE(e1 != e2);
    REQUIRE(e2 != e3);
    REQUIRE(mgr.valid(e1));
    REQUIRE(mgr.valid(e2));
    REQUIRE(mgr.valid(e3));
    REQUIRE(!mgr.valid(robcraft::engine::core::INVALID_ENTITY));
    REQUIRE(mgr.max_allocated() == 3);
}

TEST_CASE("EntityManager reuses destroyed entity IDs", "[entity]") {
    robcraft::engine::core::EntityManager mgr;

    auto e1 = mgr.create();
    auto e2 = mgr.create();
    mgr.destroy(e1);
    auto e3 = mgr.create();

    REQUIRE(e3 == e1);
    REQUIRE(mgr.valid(e3));
}

TEST_CASE("EntityManager ignores invalid destroys", "[entity]") {
    robcraft::engine::core::EntityManager mgr;
    mgr.destroy(99999);  // should not crash
    mgr.destroy(robcraft::engine::core::INVALID_ENTITY);
    REQUIRE(true);
}

TEST_CASE("EntityManager reset clears state", "[entity]") {
    robcraft::engine::core::EntityManager mgr;
    mgr.create();
    mgr.create();
    mgr.reset();
    auto e = mgr.create();
    REQUIRE(e == 1);
}
