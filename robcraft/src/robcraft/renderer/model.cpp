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

#include "robcraft/renderer/model.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

#include "robcraft/engine/core/data_path.hpp"
#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/io/path_utils.hpp"
#include "robcraft/renderer/gltf_loader.hpp"
#include "robcraft/renderer/obj_loader.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::core;
using namespace robcraft::engine::io;

namespace {

/**
 * @brief Detects whether a path is a glTF model by file extension.
 * @param path Model file path.
 * @return True for ".gltf" or ".glb" suffixes (case-insensitive).
 */
bool is_gltf_path(const std::string& path) {
    if (path.size() < 5) return false;
    std::string lower;
    lower.reserve(path.size());
    for (char c : path)
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    return lower.rfind(".gltf") == lower.size() - 5 ||
           (path.size() >= 4 && lower.rfind(".glb") == lower.size() - 4);
}

}  // namespace

Model::~Model() {}

Model::Model(Model&& o) noexcept
    : submeshes_(std::move(o.submeshes_)),
      skin_(std::move(o.skin_)),
      animations_(std::move(o.animations_)),
      bounds_min_(o.bounds_min_),
      bounds_max_(o.bounds_max_) {}

Model& Model::operator=(Model&& o) noexcept {
    if (this != &o) {
        this->submeshes_ = std::move(o.submeshes_);
        this->skin_ = std::move(o.skin_);
        this->animations_ = std::move(o.animations_);
        this->bounds_min_ = o.bounds_min_;
        this->bounds_max_ = o.bounds_max_;
    }
    return *this;
}

Model Model::load(const std::string& path) {
    ObjData data;
    if (!load_obj_file(path, data)) {
        auto log = get_logger();
        log->error("Model::load: failed to parse OBJ: {}", path);
        return Model();
    }

    // Resolve the MTL directory for texture lookups (same rules as the loader).
    std::string mtl_path;
    {
        std::ifstream f(path);
        std::stringstream ss;
        ss << f.rdbuf();
        std::istringstream obj(ss.str());
        std::string line;
        while (std::getline(obj, line)) {
            std::istringstream ls(line);
            std::string tag;
            std::string name;
            if (ls >> tag && tag == "mtllib" && ls >> name) {
                mtl_path = directory_of(path) + name;
                break;
            }
        }
    }
    if (mtl_path.empty()) mtl_path = sibling_mtl_path(path);
    const std::string mtl_dir = directory_of(mtl_path);

    // Distinct material indices in order of first appearance.
    std::vector<int> material_order;
    for (int mi : data.vertex_material) {
        if (std::find(material_order.begin(), material_order.end(), mi) == material_order.end())
            material_order.push_back(mi);
    }

    Model m;
    m.set_bounds(data.bounds_min, data.bounds_max);

    // Models with UVs but no map_Kd get a procedural albedo derived from the Kd color.
    bool has_uvs = false;
    for (const auto& v : data.vertices) {
        if (v.u != 0.0f || v.v != 0.0f) {
            has_uvs = true;
            break;
        }
    }

    if (material_order.size() <= 1) {
        // Single material group: upload everything as one submesh.
        ModelMesh sub;
        sub.mesh.upload(data.vertices, data.indices);
        int mi = material_order.empty() ? -1 : material_order[0];
        if (mi >= 0 && mi < static_cast<int>(data.materials.size())) {
            const auto& mat = data.materials[mi];
            if (!mat.map_kd.empty()) {
                sub.albedo = Texture::create_2d(mtl_dir + mat.map_kd);
            } else if (has_uvs) {
                sub.albedo = Texture::create_procedural(mat.name.empty() ? "mat" : mat.name);
            }
            if (!mat.map_bump.empty()) sub.normal = Texture::create_2d(mtl_dir + mat.map_bump);
        }
        m.submeshes_.push_back(std::move(sub));
        return m;
    }

    for (int mi : material_order) {
        // Collect the triangles belonging to this material (grouped by first corner).
        std::vector<int> remap(data.vertices.size(), -1);
        std::vector<Vertex> verts;
        std::vector<GLuint> idxs;
        for (size_t i = 0; i < data.indices.size(); i += 3) {
            int lead = static_cast<int>(data.indices[i]);
            if (data.vertex_material[lead] != mi) continue;
            for (int k = 0; k < 3; ++k) {
                int vidx = static_cast<int>(data.indices[i + k]);
                if (remap[vidx] < 0) {
                    remap[vidx] = static_cast<int>(verts.size());
                    verts.push_back(data.vertices[vidx]);
                }
                idxs.push_back(static_cast<GLuint>(remap[vidx]));
            }
        }
        if (verts.empty()) continue;
        ModelMesh sub;
        sub.mesh.upload(verts, idxs);
        if (mi >= 0 && mi < static_cast<int>(data.materials.size())) {
            const auto& mat = data.materials[mi];
            if (!mat.map_kd.empty()) {
                sub.albedo = Texture::create_2d(mtl_dir + mat.map_kd);
            } else if (has_uvs) {
                sub.albedo = Texture::create_procedural(mat.name.empty() ? "mat" : mat.name);
            }
            if (!mat.map_bump.empty()) sub.normal = Texture::create_2d(mtl_dir + mat.map_bump);
        }
        m.submeshes_.push_back(std::move(sub));
    }
    return m;
}

Model Model::load_skinned(const std::string& path) {
    GltfModelData g;
    if (!load_gltf_file(path, g)) {
        auto log = get_logger();
        log->error("Model::load_skinned: failed to parse glTF: {}", path);
        return Model();
    }
    Model m;
    m.skin_ = std::move(g.skin);
    m.animations_ = std::move(g.animations);
    m.set_bounds(g.bounds_min, g.bounds_max);
    // Clamp joint indices to the shader's 64-slot uniform limit; anything beyond
    // would read out of bounds in the shader.
    const size_t kMaxJoints = 64;
    for (auto& md : g.meshes) {
        for (auto& j : md.joint_indices) {
            if (j >= kMaxJoints) j = 0;
        }
    }
    for (auto& md : g.meshes) {
        ModelMesh sub;
        sub.mesh.upload(md.vertices, md.indices, std::vector<float>{}, md.joint_weights,
                        md.joint_indices);
        if (md.material >= 0 && md.material < static_cast<int>(g.material_colors.size())) {
            sub.albedo =
                Texture::create_procedural("robot_material_" + std::to_string(md.material));
        }
        m.submeshes_.push_back(std::move(sub));
    }
    return m;
}

std::shared_ptr<Model> ModelCache::get(const std::string& path) {
    std::string resolved = robcraft::engine::core::resolve_data_path(path);
    auto it = this->cache_.find(resolved);
    if (it != this->cache_.end()) return it->second;
    auto m = std::make_shared<Model>(is_gltf_path(resolved) ? Model::load_skinned(resolved)
                                                            : Model::load(resolved));
    this->cache_[resolved] = m;
    return m;
}

std::shared_ptr<Model> ModelCache::get_skinned(const std::string& path) {
    std::string resolved = robcraft::engine::core::resolve_data_path(path);
    auto it = this->cache_.find(resolved);
    if (it != this->cache_.end()) return it->second;
    auto m = std::make_shared<Model>(Model::load_skinned(resolved));
    this->cache_[resolved] = m;
    return m;
}

}  // namespace robcraft::renderer
