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

#pragma once

#include <vector>

#include "robcraft/engine/core/entity.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::engine::ecs {

using namespace robcraft::engine::core;
using namespace robcraft::engine::world;

/**
 * @brief Returns the entities to draw: collider entities plus "doodads"
 *        (transformed, named entities without a collider, excluding lights).
 * @param world The world to query.
 * @return Drawable entity ids.
 */
std::vector<Entity> collect_scene_entities(const World& world);

}  // namespace robcraft::engine::ecs
