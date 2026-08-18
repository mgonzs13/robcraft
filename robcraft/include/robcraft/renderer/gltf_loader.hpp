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

#include <cstdint>
#include <string>
#include <vector>

#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/mesh.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

/** @brief One skinned mesh submesh: geometry + skinning data. */
struct GltfMeshData {
    /** @brief Positions/normals/colors/UVs/tangent (Vertex as used by OBJ loader). */
    std::vector<Vertex> vertices;
    /** @brief Triangle index list. */
    std::vector<unsigned int> indices;
    /** @brief Joint weights, 4 floats per vertex. */
    std::vector<float> joint_weights;
    /** @brief Joint indices, 4 u16 per vertex. */
    std::vector<uint16_t> joint_indices;
    /** @brief Material index into the glTF materials array, or -1. */
    int material = -1;
};

/** @brief Joint hierarchy + inverse bind matrices for skinning. */
struct GltfSkin {
    /** @brief Joint node indices in skin order. */
    std::vector<int> joint_nodes;
    /** @brief Inverse bind matrices per joint. */
    std::vector<Mat4> inverse_bind;
    /** @brief Parent node index per joint (-1 for root). */
    std::vector<int> parent;
    /** @brief Local TRS as Mat4 per node (bind pose, original model frame). */
    std::vector<Mat4> node_local;
    /** @brief Combined-geometry center (original frame) of the normalization
     *  vertex' = (vertex - center) * scale. Mirrors GltfModelData.center. */
    Vec3 center;
    /** @brief Combined-geometry scale (max extent -> 1.0). Mirrors
     *  GltfModelData.scale. */
    double scale = 1.0;
    /** @brief True if skin data was loaded. */
    bool valid = false;
};

/** @brief One animation clip: per-node TRS keyframe tracks. */
struct GltfAnimation {
    /** @brief Track path kind. */
    enum Path { Translation = 0, Rotation = 1, Scale = 2 };

    /** @brief One channel: node, path, keyframes. */
    struct Track {
        /** @brief Node index the track targets. */
        int node = 0;
        /** @brief Path kind (Translation/Rotation/Scale). */
        int path = 0;
        /** @brief Keyframe times (seconds). */
        std::vector<float> times;
        /** @brief Keyframe values: 3 floats/key (T/S) or 4 floats/key (R, wxyz). */
        std::vector<float> values;
    };

    /** @brief Clip name (e.g. "Walk"). */
    std::string name;
    /** @brief Tracks in channel order. */
    std::vector<Track> tracks;
    /** @brief Total duration in seconds. */
    float duration = 0.0f;
};

/** @brief Parsed glTF document. */
struct GltfModelData {
    /** @brief Combined geometry transform: vertex' = (vertex - center) * scale.
     *  Applied to all submeshes and to skin bind/animation matrices. */
    Vec3 center;
    /** @brief Combined geometry scale (max extent -> 1.0). */
    double scale = 1.0;
    /** @brief Unit-normalized lower bound corner (vertex' = (v - center) * scale). */
    Vec3 bounds_min;
    /** @brief Unit-normalized upper bound corner. */
    Vec3 bounds_max;
    /** @brief Mesh submeshes (one per primitive). */
    std::vector<GltfMeshData> meshes;
    /** @brief Base-color (baseColorFactor RGB) per material, default white. */
    std::vector<Vec3> material_colors;
    /** @brief Skeleton (may be !valid). */
    GltfSkin skin;
    /** @brief Animation clips. */
    std::vector<GltfAnimation> animations;
};

/** @brief Parses glTF 2.0 JSON text into CPU data.
 *  @param json_text The glTF JSON document.
 *  @param out Parsed model.
 *  @return True on success. */
bool load_gltf_from_memory(const std::string& json_text, GltfModelData& out);

/** @brief Parses a .gltf file.
 *  @param path File path.
 *  @param out Parsed model.
 *  @return True on success. */
bool load_gltf_file(const std::string& path, GltfModelData& out);

}  // namespace robcraft::renderer
