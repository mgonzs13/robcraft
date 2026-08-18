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
#include <string>
#include <vector>

#include "robcraft/renderer/gltf_loader.hpp"

using namespace robcraft::renderer;

TEST_CASE("farm animal glTF models load with skin and Idle animation", "[animals][gltf_loader]") {
    const std::vector<std::string> animal_names = {"Cow", "Horse", "Llama", "Pig",
                                                   "Pug", "Sheep", "Zebra"};

    for (const auto& name : animal_names) {
        robcraft::renderer::GltfModelData data;
        REQUIRE(robcraft::renderer::load_gltf_file("assets/models/animals/gltf/" + name + ".gltf",
                                                   data));
        REQUIRE(!data.meshes.empty());
        REQUIRE(data.skin.valid);
        REQUIRE(!data.skin.joint_nodes.empty());
        REQUIRE(data.skin.joint_nodes.size() <= 64);
        REQUIRE(!data.animations.empty());

        bool has_idle = false;
        for (const auto& a : data.animations) {
            if (a.name == "Idle") {
                has_idle = true;
                break;
            }
        }
        REQUIRE(has_idle);

        REQUIRE(data.meshes[0].joint_weights.size() == data.meshes[0].vertices.size() * 4);
    }
}