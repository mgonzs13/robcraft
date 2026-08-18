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

#include "robcraft/engine/math/mat4.hpp"

namespace robcraft::engine::lighting {
struct SceneLighting;
}

namespace robcraft::engine::world {
class World;
}

namespace robcraft::renderer {

class Shader;
class Texture;

using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

/** @brief Uploads the sun/ambient lighting uniforms.
 *  @param shader The active shader.
 *  @param lighting The world's scene lighting. */
void upload_scene_lighting(const Shader& shader, const SceneLighting& lighting);

/** @brief Uploads up to 8 point lights from the world's PointLight components.
 *  @param shader The active shader.
 *  @param world The world to gather lights from. */
void upload_point_lights(const Shader& shader, const World& world);

/** @brief Uploads bone matrices for skinned model drawing.
 *  @param shader The active shader.
 *  @param matrices Joint matrices (<= 64). */
void upload_bone_matrices(const Shader& shader, const std::vector<Mat4>& matrices);

/** @brief Binds the terrain albedo/normal texture arrays.
 *  @param shader The active shader.
 *  @param albedo 5-layer albedo array (or invalid for flat-color fallback).
 *  @param normal 5-layer normal array (or invalid).
 *  @param use_splat True to enable splat sampling. */
void bind_terrain_textures(const Shader& shader, const Texture& albedo, const Texture& normal,
                           bool use_splat);

/** @brief Binds a model albedo/normal texture pair.
 *  @param shader The active shader.
 *  @param albedo Albedo texture (or invalid for vertex-color rendering).
 *  @param normal Normal texture (or invalid). */
void bind_model_textures(const Shader& shader, const Texture& albedo, const Texture& normal);

/** @brief Sets all water shader defaults (normal map, reflection off).
 *  @param shader The active shader. */
void set_water_defaults(const Shader& shader);

}  // namespace robcraft::renderer
