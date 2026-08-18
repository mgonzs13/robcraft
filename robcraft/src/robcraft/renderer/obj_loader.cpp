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

#include "robcraft/renderer/obj_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>

#include "robcraft/engine/io/path_utils.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::io;
using namespace robcraft::engine::math;

namespace {

/** @brief Temporary corner reference before fan-triangulation. */
struct ObjCorner {
    /** @brief 1-based index into the position list. */
    int pos = 0;
    /** @brief 1-based index into the UV list (0 = none). */
    int uv = 0;
    /** @brief 1-based index into the normal list (0 = none). */
    int nrm = 0;
};

/**
 * @brief Parses MTL text into a material table (in newmtl order).
 * @param mtl MTL file content.
 * @return Parsed materials (empty if no newmtl lines).
 */
std::vector<ObjMaterial> parse_mtl(const std::string& mtl) {
    std::vector<ObjMaterial> materials;
    ObjMaterial cur;
    bool have_cur = false;
    std::istringstream ss(mtl);
    std::string line;
    while (std::getline(ss, line)) {
        std::string::size_type hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag == "newmtl") {
            if (have_cur) materials.push_back(cur);
            cur = ObjMaterial();
            ls >> cur.name;
            have_cur = true;
        } else if (tag == "Kd") {
            ls >> cur.kd_r >> cur.kd_g >> cur.kd_b;
        } else if (tag == "map_Kd") {
            if (cur.map_kd.empty()) ls >> cur.map_kd;
        } else if (tag == "map_Bump" || tag == "map_Ka") {
            if (cur.map_bump.empty()) ls >> cur.map_bump;
        }
    }
    if (have_cur) materials.push_back(cur);
    return materials;
}

}  // namespace

bool load_obj_from_memory_with_mtl(const std::string& content, const std::string& mtl,
                                   ObjData& out) {
    const std::vector<ObjMaterial> materials = parse_mtl(mtl);
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<std::pair<float, float>> uvs;
    std::vector<std::array<unsigned char, 3>> colors;
    std::vector<bool> pos_has_color;
    std::vector<ObjCorner> corners;
    std::vector<int> corner_material;

    auto material_index = [&materials](const std::string& name) {
        for (size_t i = 0; i < materials.size(); ++i)
            if (materials[i].name == name) return static_cast<int>(i);
        return -1;
    };
    int current_material = -1;

    std::istringstream ss(content);
    std::string line;
    while (std::getline(ss, line)) {
        // OBJ treats '#' as a comment running to the end of the line.
        std::string::size_type hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag == "v") {
            float x, y, z;
            float r = 1.0f, g = 1.0f, b = 1.0f;
            ls >> x >> y >> z;
            bool has_color = static_cast<bool>(ls >> r >> g >> b);
            positions.push_back(Vec3(x, y, z));
            colors.push_back(
                has_color ? std::array<unsigned char, 3>{static_cast<unsigned char>(r * 255.0f),
                                                         static_cast<unsigned char>(g * 255.0f),
                                                         static_cast<unsigned char>(b * 255.0f)}
                          : std::array<unsigned char, 3>{255, 255, 255});
            pos_has_color.push_back(has_color);
        } else if (tag == "vt") {
            float u, v;
            ls >> u >> v;
            uvs.push_back({u, v});
        } else if (tag == "vn") {
            float x, y, z;
            ls >> x >> y >> z;
            normals.push_back(Vec3(x, y, z));
        } else if (tag == "usemtl") {
            std::string name;
            if (ls >> name) current_material = material_index(name);
        } else if (tag == "f") {
            std::string tok;
            std::vector<ObjCorner> face;
            while (ls >> tok) {
                ObjCorner c;
                std::stringstream ts(tok);
                std::string part;
                int idx = 0;
                while (std::getline(ts, part, '/')) {
                    if (!part.empty()) {
                        try {
                            int val = std::stoi(part);  // OBJ is 1-based; keep raw, resolve below
                            if (idx == 0)
                                c.pos = val;
                            else if (idx == 1)
                                c.uv = val;
                            else if (idx == 2)
                                c.nrm = val;
                        } catch (const std::exception&) {
                            c.pos = 0;  // malformed token: mark corner invalid
                        }
                    }
                    idx++;
                }
                face.push_back(c);
            }
            if (face.size() < 3) continue;
            // Reject the whole face if any corner references an out-of-range position.
            bool face_valid = true;
            for (const auto& c : face) {
                if (c.pos <= 0 || c.pos > static_cast<int>(positions.size())) {
                    face_valid = false;
                    break;
                }
            }
            if (!face_valid) continue;
            // Out-of-range UV/normal indices degrade to "none".
            for (auto& c : face) {
                if (c.uv <= 0 || c.uv > static_cast<int>(uvs.size())) c.uv = 0;
                if (c.nrm <= 0 || c.nrm > static_cast<int>(normals.size())) c.nrm = 0;
            }
            // Fan-triangulate into corners list (shared vertices keep smoothing).
            for (size_t i = 1; i + 1 < face.size(); ++i) {
                corners.push_back(face[0]);
                corners.push_back(face[i]);
                corners.push_back(face[i + 1]);
                corner_material.push_back(current_material);
                corner_material.push_back(current_material);
                corner_material.push_back(current_material);
            }
        }
    }

    out.vertices.clear();
    out.indices.clear();
    out.materials = materials;
    out.vertex_material.clear();

    // Unit-normalize: center at origin, scale so max extent == 1.0.
    Vec3 min_p(1e30, 1e30, 1e30), max_p(-1e30, -1e30, -1e30);
    for (const auto& p : positions) {
        min_p.x = std::min(min_p.x, p.x);
        max_p.x = std::max(max_p.x, p.x);
        min_p.y = std::min(min_p.y, p.y);
        max_p.y = std::max(max_p.y, p.y);
        min_p.z = std::min(min_p.z, p.z);
        max_p.z = std::max(max_p.z, p.z);
    }
    Vec3 center = (min_p + max_p) * 0.5;
    double extent = std::max({max_p.x - min_p.x, max_p.y - min_p.y, max_p.z - min_p.z});
    double scale = extent > 0.0 ? 1.0 / extent : 1.0;

    out.bounds_min = (min_p - center) * scale;
    out.bounds_max = (max_p - center) * scale;

    auto corner_index = [&](const ObjCorner& c, int mi) {
        int p = c.pos - 1;
        Vec3 pos = (positions[p] - center) * scale;
        Vec3 nrm = c.nrm > 0 ? normals[c.nrm - 1].normalized() : Vec3(0, 1, 0);
        float u = c.uv > 0 ? uvs[c.uv - 1].first : 0.0f;
        float v = c.uv > 0 ? uvs[c.uv - 1].second : 0.0f;
        unsigned char r = colors[p][0], g = colors[p][1], b = colors[p][2];
        if (!pos_has_color[p] && mi >= 0 && mi < static_cast<int>(materials.size())) {
            const auto& mat = materials[mi];
            if (mat.kd_r != 1.0f || mat.kd_g != 1.0f || mat.kd_b != 1.0f) {
                r = static_cast<unsigned char>(mat.kd_r * 255.0f);
                g = static_cast<unsigned char>(mat.kd_g * 255.0f);
                b = static_cast<unsigned char>(mat.kd_b * 255.0f);
            }
        }
        out.vertices.push_back({static_cast<float>(pos.x), static_cast<float>(pos.y),
                                static_cast<float>(pos.z), static_cast<float>(nrm.x),
                                static_cast<float>(nrm.y), static_cast<float>(nrm.z), r / 255.0f,
                                g / 255.0f, b / 255.0f, u, v, 1.0f, 0.0f, 0.0f});
        out.vertex_material.push_back(mi);
        return static_cast<unsigned int>(out.vertices.size() - 1);
    };

    for (size_t i = 0; i + 2 < corners.size(); i += 3) {
        unsigned int a = corner_index(corners[i], corner_material[i]);
        unsigned int b = corner_index(corners[i + 1], corner_material[i + 1]);
        unsigned int c = corner_index(corners[i + 2], corner_material[i + 2]);
        out.indices.push_back(a);
        out.indices.push_back(b);
        out.indices.push_back(c);
    }

    // Compute tangents from face UV deltas (averaged per vertex).
    std::vector<Vec3> tangents(out.vertices.size());
    std::vector<int> tangent_count(out.vertices.size(), 0);
    for (size_t i = 0; i < out.indices.size(); i += 3) {
        const auto& v0 = out.vertices[out.indices[i]];
        const auto& v1 = out.vertices[out.indices[i + 1]];
        const auto& v2 = out.vertices[out.indices[i + 2]];
        Vec3 e1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        Vec3 e2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        float du1 = v1.u - v0.u, dv1 = v1.v - v0.v;
        float du2 = v2.u - v0.u, dv2 = v2.v - v0.v;
        float det = du1 * dv2 - du2 * dv1;
        Vec3 t(1, 0, 0);
        if (std::fabs(det) > 1e-8f) {
            t = (e1 * dv2 - e2 * dv1) / det;
        }
        for (int k = 0; k < 3; ++k) {
            tangents[out.indices[i + k]] += t;
            tangent_count[out.indices[i + k]]++;
        }
    }
    for (size_t i = 0; i < out.vertices.size(); ++i) {
        if (tangent_count[i] > 0) {
            Vec3 t = tangents[i] / static_cast<double>(tangent_count[i]);
            Vec3 n(out.vertices[i].nx, out.vertices[i].ny, out.vertices[i].nz);
            // Gram-Schmidt: make the tangent perpendicular to the vertex normal.
            Vec3 gs = t - n * n.dot(t);
            if (gs.length_sq() < 1e-12) {
                // Degenerate tangent: pick any vector orthogonal to the normal.
                Vec3 ref = std::fabs(n.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
                gs = n.cross(ref).normalized();
            } else {
                gs = gs.normalized();
            }
            out.vertices[i].tx = static_cast<float>(gs.x);
            out.vertices[i].ty = static_cast<float>(gs.y);
            out.vertices[i].tz = static_cast<float>(gs.z);
        }
    }

    // Compute face normals when the file omitted them.
    if (normals.empty()) {
        for (size_t i = 0; i < out.indices.size(); i += 3) {
            auto& a = out.vertices[out.indices[i]];
            auto& b = out.vertices[out.indices[i + 1]];
            auto& c = out.vertices[out.indices[i + 2]];
            Vec3 n = (Vec3(b.x - a.x, b.y - a.y, b.z - a.z))
                         .cross(Vec3(c.x - a.x, c.y - a.y, c.z - a.z))
                         .normalized();
            a.nx = static_cast<float>(n.x);
            a.ny = static_cast<float>(n.y);
            a.nz = static_cast<float>(n.z);
            b.nx = static_cast<float>(n.x);
            b.ny = static_cast<float>(n.y);
            b.nz = static_cast<float>(n.z);
            c.nx = static_cast<float>(n.x);
            c.ny = static_cast<float>(n.y);
            c.nz = static_cast<float>(n.z);
        }
    }

    return !out.vertices.empty();
}

bool load_obj_file(const std::string& path, ObjData& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string obj = ss.str();

    // Find the MTL: first mtllib reference, else the sibling .mtl.
    std::string mtl_path;
    {
        std::istringstream obj_is(obj);
        std::string line;
        while (std::getline(obj_is, line)) {
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

    std::string mtl;
    std::ifstream mf(mtl_path);
    if (mf.is_open()) {
        std::stringstream mss;
        mss << mf.rdbuf();
        mtl = mss.str();
    }
    return load_obj_from_memory_with_mtl(obj, mtl, out);
}

}  // namespace robcraft::renderer
