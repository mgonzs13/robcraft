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
#include <string>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/gltf_loader.hpp"
#include "robcraft/renderer/model.hpp"
#include "robcraft/renderer/model_paths.hpp"

using namespace robcraft::renderer;

using Catch::Approx;

TEST_CASE("model_path_for_prefix falls back to box", "[model_paths]") {
    REQUIRE(robcraft::renderer::model_path_for_prefix("unknown") == "assets/models/box/box.obj");
    REQUIRE(robcraft::renderer::model_path_for_prefix("crate") ==
            "assets/models/space/Pickup_Crate.obj");
}

TEST_CASE("placement_spec_for_name resolves real-name prefixes", "[model_paths]") {
    auto wall = robcraft::renderer::placement_spec_for_name("wall_12");
    REQUIRE(wall != nullptr);
    REQUIRE(wall->model_path.empty());  // primitive
    REQUIRE(wall->name_prefix == "wall");

    auto couch = robcraft::renderer::placement_spec_for_name("couch_2");
    REQUIRE(couch != nullptr);
    REQUIRE(couch->name_prefix == "couch");
    REQUIRE(couch->model_path == "assets/models/interior/Couch_Large1.obj");

    auto light = robcraft::renderer::placement_spec_for_name("light_floor_1");
    REQUIRE(light != nullptr);
    REQUIRE(light->name_prefix == "light_floor");
    REQUIRE(light->model_path == "assets/models/interior/Light_Floor1.obj");

    auto shelf = robcraft::renderer::placement_spec_for_name("shelf_3");
    REQUIRE(shelf != nullptr);
    REQUIRE(shelf->name_prefix == "shelf");
    REQUIRE(shelf->model_path == "assets/models/interior/Shelf_1.obj");

    auto tree = robcraft::renderer::placement_spec_for_name("tree_3");
    REQUIRE(tree != nullptr);
    REQUIRE(tree->model_path == "assets/models/nature/CommonTree_3.obj");

    auto tree_alias = robcraft::renderer::placement_spec_for_name("tree");
    REQUIRE(tree_alias != nullptr);
    REQUIRE(tree_alias->name_prefix == "tree");
    REQUIRE(tree_alias->model_path == "assets/models/nature/CommonTree_1.obj");

    auto tree_2 = robcraft::renderer::placement_spec_for_name("tree_2_7");
    REQUIRE(tree_2 != nullptr);
    REQUIRE(tree_2->name_prefix == "tree_2");
    REQUIRE(tree_2->model_path == "assets/models/nature/CommonTree_2.obj");

    auto pine = robcraft::renderer::placement_spec_for_name("pine_1_5");
    REQUIRE(pine != nullptr);
    REQUIRE(pine->name_prefix == "pine_1");
    REQUIRE(pine->model_path == "assets/models/nature/Pine_1.obj");

    auto twisted = robcraft::renderer::placement_spec_for_name("twisted_4_9");
    REQUIRE(twisted != nullptr);
    REQUIRE(twisted->name_prefix == "twisted_4");
    REQUIRE(twisted->model_path == "assets/models/nature/TwistedTree_4.obj");

    auto dead = robcraft::renderer::placement_spec_for_name("dead_5_2");
    REQUIRE(dead != nullptr);
    REQUIRE(dead->name_prefix == "dead_5");
    REQUIRE(dead->model_path == "assets/models/nature/DeadTree_5.obj");

    auto bush = robcraft::renderer::placement_spec_for_name("bush_2");
    REQUIRE(bush != nullptr);
    REQUIRE(bush->model_path == "assets/models/nature/Bush_Common_Flowers.obj");

    auto base = robcraft::renderer::placement_spec_for_name("space_base_1");
    REQUIRE(base != nullptr);
    REQUIRE(base->model_path == "assets/models/space/Base_Large.obj");
    REQUIRE(base->multi_tile);
    auto fp = robcraft::renderer::placement_footprint_cells(base, 2.0);
    REQUIRE(fp.first >= 5);
    REQUIRE(fp.second >= 4);

    auto mike = robcraft::renderer::placement_spec_for_name("robot_mike_1");
    REQUIRE(mike != nullptr);
    REQUIRE(mike->model_path == "assets/models/mech/gltf/Mike.gltf");

    auto cow = robcraft::renderer::placement_spec_for_name("cow_7");
    REQUIRE(cow != nullptr);
    REQUIRE(cow->name_prefix == "cow");
    REQUIRE(cow->model_path == "assets/models/animals/gltf/Cow.gltf");
    REQUIRE(cow->solid);

    auto zebra = robcraft::renderer::placement_spec_for_name("zebra_1");
    REQUIRE(zebra != nullptr);
    REQUIRE(zebra->model_path == "assets/models/animals/gltf/Zebra.gltf");
}

TEST_CASE("mesh_label_for_name maps animal prefixes", "[model_paths]") {
    REQUIRE(robcraft::renderer::mesh_label_for_name("cow_3") == "cow");
    REQUIRE(robcraft::renderer::mesh_label_for_name("horse_2") == "horse");
    REQUIRE(robcraft::renderer::mesh_label_for_name("llama_1") == "llama");
    REQUIRE(robcraft::renderer::mesh_label_for_name("pig_9") == "pig");
    REQUIRE(robcraft::renderer::mesh_label_for_name("pug_4") == "pug");
    REQUIRE(robcraft::renderer::mesh_label_for_name("sheep_6") == "sheep");
    REQUIRE(robcraft::renderer::mesh_label_for_name("zebra_1") == "zebra");
}

TEST_CASE("placement_ground_offset falls back to ground_frac", "[model_paths]") {
    auto tree = robcraft::renderer::placement_spec_for_name("tree_2");
    REQUIRE(tree != nullptr);
    robcraft::engine::math::Vec3 scale(0.8, 0.8, 0.8);
    // No model available (nullptr): spec ground_frac is used.
    REQUIRE(robcraft::renderer::placement_ground_offset(tree, nullptr, scale) ==
            Approx(0.5f * 0.8f));
    // No spec, no model: centered fallback.
    REQUIRE(robcraft::renderer::placement_ground_offset(nullptr, nullptr, scale) == Approx(0.4f));
}

TEST_CASE("all model-backed placement specs use uniform scale", "[model_paths]") {
    const char* kAll[] = {
        "robot_george", "robot_leela",   "robot_mike",  "robot_stan",   "cow",
        "horse",        "llama",         "pig",         "pug",          "sheep",
        "zebra",        "tree",          "tree_1",      "tree_2",       "tree_3",
        "tree_4",       "tree_5",        "pine_1",      "pine_2",       "pine_3",
        "pine_4",       "pine_5",        "twisted_1",   "twisted_2",    "twisted_3",
        "twisted_4",    "twisted_5",     "dead_1",      "dead_2",       "dead_3",
        "dead_4",       "dead_5",        "bush",        "bush_2",       "rock_1",
        "rock_2",       "rock_3",        "rock_4",      "rock_large_1", "rock_large_2",
        "rock_large_3", "moon_rock",     "moon_rock_2", "moon_rock_3",  "moon_rock_large",
        "space_base",   "geodesic_dome", "solar_panel", "bed",          "chair",
        "couch",        "light_floor",   "shelf",       "table",        "rock",
        "base",         "box",           "crate",       "cone"};
    for (const char* prefix : kAll) {
        const PlacementSpec* spec = robcraft::renderer::placement_spec_for_prefix(prefix);
        REQUIRE(spec != nullptr);
        REQUIRE_FALSE(spec->model_path.empty());
        REQUIRE(spec->base_scale.x > 0.0f);
        REQUIRE(spec->base_scale.x == Approx(spec->base_scale.y));
        REQUIRE(spec->base_scale.y == Approx(spec->base_scale.z));
    }
}

TEST_CASE("collider_half_extents falls back to a scale cube", "[model_paths]") {
    robcraft::engine::math::Vec3 scale(2.0, 3.0, 4.0);
    // No model available (nullptr): half extents are half the scale per axis.
    auto half = robcraft::renderer::collider_half_extents(nullptr, scale);
    REQUIRE(half.x == Approx(1.0));
    REQUIRE(half.y == Approx(1.5));
    REQUIRE(half.z == Approx(2.0));
}

TEST_CASE("key placement specs use expected real-world sizes", "[model_paths]") {
    struct Case {
        const char* prefix;
        float expected;
    };
    const Case kCases[] = {
        {"robot_mike", 1.6f},   // drawn ~1.6 x 1.0 x 0.31 m (human-like)
        {"cow", 2.5f},          // drawn ~0.61 x 1.40 x 2.50 m
        {"bed", 2.0f},          // drawn ~1.00 x 0.69 x 2.00 m
        {"chair", 0.9f},        // drawn ~0.39 x 0.90 x 0.42 m
        {"tree_3", 8.0f},       // drawn ~3.45 x 8.00 x 3.60 m
        {"space_base", 12.0f},  // drawn ~12.00 x 6.99 x 12.00 m
        {"pug", 0.6f},          // drawn ~0.25 x 0.45 x 0.60 m
        {"crate", 1.0f},        // drawn ~0.97 x 0.95 x 1.00 m
    };
    for (const Case& c : kCases) {
        const PlacementSpec* spec = robcraft::renderer::placement_spec_for_prefix(c.prefix);
        REQUIRE(spec != nullptr);
        REQUIRE(spec->base_scale.x == Approx(c.expected));
        REQUIRE(spec->base_scale.y == Approx(c.expected));
        REQUIRE(spec->base_scale.z == Approx(c.expected));
    }
}
