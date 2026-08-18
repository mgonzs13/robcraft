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

#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/renderer/texture.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::world;

/** @brief Owns the loadable terrain/building textures and reloads them at a new size. */
struct TexturePack {
    /** @brief Current texture size in pixels (256, 512, or 1024). */
    int size = 256;
    /** @brief Terrain splat albedo layer array. */
    Texture terrain_albedo;
    /** @brief Terrain splat normal layer array. */
    Texture terrain_normal;
    /** @brief Building wall albedo. */
    Texture wall_albedo;
    /** @brief Building wall normal. */
    Texture wall_normal;
    /** @brief Building floor albedo. */
    Texture floor_albedo;
    /** @brief Building floor normal. */
    Texture floor_normal;
    /** @brief True when terrain splat textures loaded successfully. */
    bool use_splat = false;

    /** @brief Destroys and reloads all textures at the given size.
     *  @param new_size Texture size in pixels (256, 512, or 1024). */
    void load(int new_size);

    /** @brief Destroys all textures. */
    void destroy();
};

}  // namespace robcraft::renderer
