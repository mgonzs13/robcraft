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

#include <string>

#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::renderer {

class Model;

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

/**
 * @brief Creates a differential-drive robot with a standard sensor suite.
 * @param world The world to create the entity in.
 * @param prefix Name prefix; the entity is named "<prefix>_<id>".
 * @param wx World-space x position.
 * @param wz World-space z position.
 * @param model Optional model used for precise ground placement; when null the
 *        placement spec's ground fraction is used.
 * @param include_lidar Whether to attach a LiDAR sensor.
 * @return The created entity.
 * @note GPS is intentionally omitted to match editor-placed robots; the demo
 *       world adds it explicitly.
 */
Entity create_robot(World& world, const std::string& prefix, double wx, double wz,
                    const Model* model = nullptr, bool include_lidar = true);

}  // namespace robcraft::renderer
