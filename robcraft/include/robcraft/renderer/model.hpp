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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/gltf_loader.hpp"
#include "robcraft/renderer/mesh.hpp"
#include "robcraft/renderer/texture.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

/** @brief One drawable submesh with an optional albedo/normal texture. */
struct ModelMesh {
    /** @brief GPU mesh. */
    Mesh mesh;
    /** @brief Albedo texture (may be invalid → vertex color). */
    Texture albedo;
    /** @brief Normal texture (may be invalid). */
    Texture normal;
};

/** @brief A 3D model: one or more submeshes, move-only RAII. */
class Model {
public:
    Model() = default;
    ~Model();
    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    Model(Model&& o) noexcept;
    Model& operator=(Model&& o) noexcept;

    /** @brief Loads a model from an OBJ file (creates GPU meshes + textures).
     *  @param path OBJ file path.
     *  @return Model, or empty if load failed. */
    static Model load(const std::string& path);

    /** @brief Loads a skinned model from a glTF file (GPU meshes + skeleton).
     *  @param path glTF file path.
     *  @return Model (with skeleton), or empty on failure. */
    static Model load_skinned(const std::string& path);

    /** @return Submeshes in draw order. */
    const std::vector<ModelMesh>& submeshes() const { return this->submeshes_; }
    /** @return True if at least one submesh exists. */
    bool valid() const { return !this->submeshes_.empty(); }
    /** @return True if the model has a skeleton. */
    bool skinned() const { return this->skin_.valid; }
    /** @return The model skeleton. */
    const GltfSkin& skin_ref() const { return this->skin_; }
    /** @return Animation clips (glTF models only). */
    const std::vector<GltfAnimation>& animations() const { return this->animations_; }
    /** @return Axis-aligned bounds in unit-normalized space (OBJ models only). */
    const Vec3& bounds_min() const { return this->bounds_min_; }
    /** @return Axis-aligned bounds in unit-normalized space (OBJ models only). */
    const Vec3& bounds_max() const { return this->bounds_max_; }
    /** @brief Sets the unit-normalized axis-aligned bounds of the model.
     *  @param mn Lower corner.
     *  @param mx Upper corner. */
    void set_bounds(const Vec3& mn, const Vec3& mx) {
        this->bounds_min_ = mn;
        this->bounds_max_ = mx;
    }

private:
    /** @brief Submeshes in draw order. */
    std::vector<ModelMesh> submeshes_;
    /** @brief Skeleton (empty unless loaded from glTF). */
    GltfSkin skin_;
    /** @brief Animation clips (glTF only). */
    std::vector<GltfAnimation> animations_;
    /** @brief Lower bound corner in unit-normalized space (OBJ only). */
    Vec3 bounds_min_;
    /** @brief Upper bound corner in unit-normalized space (OBJ only). */
    Vec3 bounds_max_;
};

/** @brief Process-wide cache of loaded models keyed by path. */
class ModelCache {
public:
    /** @brief Returns the cached model for a path, loading it on first use.
     *  glTF paths (.gltf/.glb) are loaded as skinned models; everything else
     *  is loaded as an OBJ.
     *  @param path Model file path (OBJ, or glTF for skinned models).
     *  @return Shared model (may be empty if load failed). */
    std::shared_ptr<Model> get(const std::string& path);

    /** @brief Returns the cached skinned model for a path, loading it on first use.
     *  @param path glTF file path.
     *  @return Shared skinned model (may be empty if load failed). */
    std::shared_ptr<Model> get_skinned(const std::string& path);

private:
    /** @brief path → model cache. */
    std::map<std::string, std::shared_ptr<Model>> cache_;
};

}  // namespace robcraft::renderer
