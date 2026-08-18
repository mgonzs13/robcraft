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
#include <vector>

#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/mesh.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

/** @brief Material definition parsed from an MTL file. */
struct ObjMaterial {
    /** @brief Material name (newmtl). */
    std::string name;
    /** @brief Diffuse color (Kd), 0..1. */
    float kd_r = 1.0f, kd_g = 1.0f, kd_b = 1.0f;
    /** @brief Albedo texture path (map_Kd), empty if none. */
    std::string map_kd;
    /** @brief Normal texture path (map_Bump/map_Ka), empty if none. */
    std::string map_bump;
};

/** @brief CPU-side parsed OBJ geometry (vertex colors, optional UVs, materials). */
struct ObjData {
    /** @brief Triangle vertices (positions/normals/colors/UVs/tangent). */
    std::vector<Vertex> vertices;
    /** @brief Index list (all triangles). */
    std::vector<GLuint> indices;
    /** @brief Material table (parsed from MTL, in newmtl order). */
    std::vector<ObjMaterial> materials;
    /** @brief Material index per vertex (-1 = no material). Parallel to vertices. */
    std::vector<int> vertex_material;
    /** @brief Axis-aligned bounds in unit-normalized space (extent 1.0 max axis). */
    Vec3 bounds_min;
    /** @brief Axis-aligned bounds in unit-normalized space (extent 1.0 max axis). */
    Vec3 bounds_max;
};

/**
 * @brief Parses an OBJ file from disk (loading its MTL for diffuse colors).
 * @param path OBJ file path.
 * @param out Parsed geometry.
 * @return True on success.
 */
bool load_obj_file(const std::string& path, ObjData& out);
/**
 * @brief Parses OBJ content with an optional MTL content for material Kd colors.
 * @param obj OBJ text.
 * @param mtl MTL text (may be empty).
 * @param out Parsed geometry.
 * @return True on success.
 */
bool load_obj_from_memory_with_mtl(const std::string& obj, const std::string& mtl, ObjData& out);

}  // namespace robcraft::renderer
