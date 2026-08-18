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

#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/mesh.hpp"

namespace robcraft::engine::world {

using namespace robcraft::engine::world;
using namespace robcraft::engine::math;
using namespace robcraft::renderer;

class Terrain;

/** @brief CPU mesh data produced for rendering the terrain. */
struct TerrainMeshData {
    /** @brief Mesh vertices with position, normal, color, UV, and tangent. */
    std::vector<Vertex> vertices;
    /** @brief Triangle index list into vertices. */
    std::vector<unsigned int> indices;
    /** @brief Splat blend weights, 4 floats per vertex (Grass, Dirt, Rock, Sand; Snow implied). */
    std::vector<float> weights;
};

/** @brief Builds CPU mesh data for rendering the terrain.
 *  @param terrain The terrain to mesh.
 *  @return Terrain mesh data. */
TerrainMeshData build_terrain_mesh(const Terrain& terrain);

/** @brief Builds cell-border line data for the grid overlay (no diagonals).
 *  @param terrain The terrain to mesh.
 *  @return Grid line mesh data. */
TerrainMeshData build_terrain_grid(const Terrain& terrain);

/** @brief Builds per-cell water surface quads for rendering.
 *  @param terrain The terrain to mesh.
 *  @return Water surface mesh data. */
TerrainMeshData build_terrain_water_mesh(const Terrain& terrain);

}  // namespace robcraft::engine::world
