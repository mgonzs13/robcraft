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

#include "robcraft/engine/ecs/scene_entities.hpp"

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/world/world.hpp"

namespace robcraft::engine::ecs {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;

std::vector<Entity> collect_scene_entities(const World& world) {
    std::vector<Entity> out;
    auto* col_store = world.store<robcraft::engine::collision::BoxCollider>();
    auto* tf_store = world.store<robcraft::engine::ecs::Transform3D>();
    if (col_store && tf_store) {
        for (auto& [e, col] : *col_store) {
            if (tf_store->get(e)) out.push_back(e);
        }
    }
    if (tf_store) {
        for (auto& [e, tf] : *tf_store) {
            if (col_store && col_store->has(e)) continue;
            if (!world.get_component<robcraft::engine::ecs::Name>(e)) continue;
            if (world.get_component<robcraft::engine::lighting::PointLight>(e)) continue;
            out.push_back(e);
        }
    }
    return out;
}

}  // namespace robcraft::engine::ecs
