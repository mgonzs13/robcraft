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

#include "robcraft/renderer/gltf_loader.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

namespace {
/** @brief Decodes a data URI of form data:...;base64,<data>.
 *  @param uri The data URI.
 *  @param out Decoded bytes.
 *  @return True on success. */
bool decode_data_uri(const std::string& uri, std::vector<unsigned char>& out) {
    auto comma = uri.find(',');
    if (comma == std::string::npos) return false;
    std::string meta = uri.substr(0, comma);
    if (meta.find("base64") == std::string::npos) return false;
    std::string b64 = uri.substr(comma + 1);
    static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    out.clear();
    int val = 0, bits = 0;
    for (char c : b64) {
        if (c == '=') break;
        const char* p = strchr(tbl, c);
        if (!p) continue;
        val = (val << 6) | static_cast<int>(p - tbl);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<unsigned char>((val >> bits) & 0xFF));
        }
    }
    return !out.empty();
}

/** @brief Reads a sub-range of a buffer into floats (VEC3/VEC4/SCALAR).
 *  @param buf Full decoded buffer.
 *  @param offset Byte offset.
 *  @param count Element count.
 *  @param comps Components per element (3/4/1).
 *  @return Flattened floats. */
std::vector<float> read_floats(const std::vector<unsigned char>& buf, size_t offset, size_t count,
                               int comps) {
    std::vector<float> out;
    out.reserve(count * static_cast<size_t>(comps));
    const float* p = reinterpret_cast<const float*>(buf.data() + offset);
    for (size_t i = 0; i < count * static_cast<size_t>(comps); ++i) out.push_back(p[i]);
    return out;
}

/** @brief Reads u16 indices from a buffer sub-range.
 *  @param buf Full decoded buffer.
 *  @param offset Byte offset.
 *  @param count Value count.
 *  @return Flattened values. */
std::vector<unsigned int> read_u16(const std::vector<unsigned char>& buf, size_t offset,
                                   size_t count) {
    std::vector<unsigned int> out;
    out.reserve(count);
    const uint16_t* p = reinterpret_cast<const uint16_t*>(buf.data() + offset);
    for (size_t i = 0; i < count; ++i) out.push_back(static_cast<unsigned int>(p[i]));
    return out;
}

/** @brief Reads u8 indices from a buffer sub-range (JOINTS_0 is often ubyte).
 *  @param buf Full decoded buffer.
 *  @param offset Byte offset.
 *  @param count Value count.
 *  @return Flattened values. */
std::vector<unsigned int> read_u8(const std::vector<unsigned char>& buf, size_t offset,
                                  size_t count) {
    std::vector<unsigned int> out;
    out.reserve(count);
    for (size_t i = 0; i < count; ++i) out.push_back(static_cast<unsigned int>(buf[offset + i]));
    return out;
}

/** @brief Reads u32 indices from a buffer sub-range.
 *  @param buf Full decoded buffer.
 *  @param offset Byte offset.
 *  @param count Value count.
 *  @return Flattened values. */
std::vector<unsigned int> read_u32(const std::vector<unsigned char>& buf, size_t offset,
                                   size_t count) {
    std::vector<unsigned int> out;
    out.reserve(count);
    const uint32_t* p = reinterpret_cast<const uint32_t*>(buf.data() + offset);
    for (size_t i = 0; i < count; ++i) out.push_back(static_cast<unsigned int>(p[i]));
    return out;
}
}  // namespace

bool load_gltf_from_memory(const std::string& json_text, GltfModelData& out) {
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(json_text);
    } catch (...) {
        return false;
    }

    // Parse materials first: primitives reference them by index and the vertex
    // colors are derived from each material's baseColorFactor.
    out.material_colors.clear();
    for (const auto& mat : doc.value("materials", nlohmann::json::array())) {
        Vec3 color(1.0, 1.0, 1.0);
        if (mat.contains("pbrMetallicRoughness")) {
            const auto& pbr = mat["pbrMetallicRoughness"];
            if (pbr.contains("baseColorFactor") && pbr["baseColorFactor"].is_array() &&
                pbr["baseColorFactor"].size() >= 4) {
                const auto& bcf = pbr["baseColorFactor"];
                color = Vec3(bcf[0].get<double>(), bcf[1].get<double>(), bcf[2].get<double>());
            }
        }
        out.material_colors.push_back(color);
    }

    std::vector<unsigned char> buffer;
    if (doc.contains("buffers") && !doc["buffers"].empty()) {
        std::string uri = doc["buffers"][0].value("uri", "");
        if (!decode_data_uri(uri, buffer)) return false;
    }
    if (buffer.empty()) return false;

    auto accessors = doc.value("accessors", nlohmann::json::array());
    auto buffer_views = doc.value("bufferViews", nlohmann::json::array());

    auto view_bytes = [&](int bv) -> std::pair<size_t, size_t> {
        const auto& v = buffer_views[bv];
        size_t off = v.value("byteOffset", 0);
        size_t len = v.value("byteLength", 0);
        // Dense reads below assume a tightly-packed bufferView. A non-null byteStride would
        // interleave data and break that assumption; warn and continue (the mech pack has none).
        if (v.contains("byteStride") && !v["byteStride"].is_null()) {
            auto log = get_logger();
            log->warn(
                "glTF bufferView {} has byteStride {}; gltf_loader dense reads assume no "
                "interleaving, results may be incorrect",
                bv, v["byteStride"].get<size_t>());
        }
        return {off, len};
    };

    auto read_accessor_floats = [&](int acc, int& comps_out) {
        const auto& a = accessors[acc];
        int bv = a.value("bufferView", -1);
        size_t off = view_bytes(bv).first + a.value("byteOffset", 0);
        size_t count = a.value("count", 0);
        int comps = a["type"] == "VEC3" ? 3 : a["type"] == "VEC4" ? 4 : a["type"] == "VEC2" ? 2 : 1;
        comps_out = comps;
        return read_floats(buffer, off, count, comps);
    };

    // JOINTS_0 comes in ubyte (5121), ushort (5123), or float (5126); the mech pack
    // uses ubyte. Decode by componentType so the integer indices survive unchanged.
    auto read_accessor_ints = [&](int acc) {
        const auto& a = accessors[acc];
        int bv = a.value("bufferView", -1);
        size_t off = view_bytes(bv).first + a.value("byteOffset", 0);
        size_t count = a.value("count", 0) * 4;  // VEC4 per joint set
        int ct = a.value("componentType", 5126);
        if (ct == 5121) return read_u8(buffer, off, count);
        if (ct == 5123) return read_u16(buffer, off, count);
        std::vector<float> f = read_floats(buffer, off, a.value("count", 0), 4);
        std::vector<unsigned int> out;
        out.reserve(f.size());
        for (float v : f) out.push_back(static_cast<unsigned int>(v));
        return out;
    };

    out.meshes.clear();
    auto meshes = doc.value("meshes", nlohmann::json::array());
    for (const auto& m : meshes) {
        for (const auto& prim : m.value("primitives", nlohmann::json::array())) {
            GltfMeshData md;
            md.material = prim.value("material", -1);
            Vec3 vcol(1.0, 1.0, 1.0);
            if (md.material >= 0 && md.material < static_cast<int>(out.material_colors.size()))
                vcol = out.material_colors[md.material];
            auto attrs = prim["attributes"];
            int comps = 0;
            std::vector<float> pos = read_accessor_floats(attrs["POSITION"], comps);
            std::vector<float> nrm;
            if (attrs.contains("NORMAL")) nrm = read_accessor_floats(attrs["NORMAL"], comps);
            std::vector<float> uv;
            if (attrs.contains("TEXCOORD_0")) uv = read_accessor_floats(attrs["TEXCOORD_0"], comps);
            std::vector<float> jw;
            if (attrs.contains("WEIGHTS_0")) jw = read_accessor_floats(attrs["WEIGHTS_0"], comps);
            std::vector<unsigned int> ji;
            if (attrs.contains("JOINTS_0")) ji = read_accessor_ints(attrs["JOINTS_0"]);

            size_t n = pos.size() / 3;
            for (size_t i = 0; i < n; ++i) {
                Vec3 p(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
                Vec3 nr =
                    nrm.empty() ? Vec3(0, 1, 0) : Vec3(nrm[i * 3], nrm[i * 3 + 1], nrm[i * 3 + 2]);
                float u = uv.empty() ? 0.0f : uv[i * 2];
                float v = uv.empty() ? 0.0f : uv[i * 2 + 1];
                md.vertices.push_back({static_cast<float>(p.x), static_cast<float>(p.y),
                                       static_cast<float>(p.z), static_cast<float>(nr.x),
                                       static_cast<float>(nr.y), static_cast<float>(nr.z),
                                       static_cast<float>(vcol.x), static_cast<float>(vcol.y),
                                       static_cast<float>(vcol.z), u, v, 1.0f, 0.0f, 0.0f});
                if (!jw.empty()) {
                    md.joint_weights.push_back(jw[i * 4]);
                    md.joint_weights.push_back(jw[i * 4 + 1]);
                    md.joint_weights.push_back(jw[i * 4 + 2]);
                    md.joint_weights.push_back(jw[i * 4 + 3]);
                }
                if (!ji.empty()) {
                    for (int k = 0; k < 4; ++k)
                        md.joint_indices.push_back(static_cast<uint16_t>(ji[i * 4 + k]));
                }
            }

            int idx_acc = prim.value("indices", -1);
            if (idx_acc >= 0) {
                const auto& a = accessors[idx_acc];
                int bv = a.value("bufferView", -1);
                size_t off = view_bytes(bv).first + a.value("byteOffset", 0);
                size_t count = a.value("count", 0);
                int ct = a.value("componentType", 5123);
                if (ct == 5121) {
                    md.indices = read_u8(buffer, off, count);
                } else if (ct == 5125) {
                    md.indices = read_u32(buffer, off, count);
                } else {
                    md.indices = read_u16(buffer, off, count);
                }
            } else {
                for (size_t i = 0; i < n; ++i) md.indices.push_back(static_cast<unsigned int>(i));
            }

            out.meshes.push_back(std::move(md));
        }
    }

    // Combine all submeshes into ONE frame so they stay aligned with the skeleton's bind
    // matrices (per-submesh normalization would scatter each submesh to its own origin).
    Vec3 min_p(1e30, 1e30, 1e30), max_p(-1e30, -1e30, -1e30);
    for (const auto& md : out.meshes) {
        for (const auto& v : md.vertices) {
            min_p.x = std::min(min_p.x, (double)v.x);
            max_p.x = std::max(max_p.x, (double)v.x);
            min_p.y = std::min(min_p.y, (double)v.y);
            max_p.y = std::max(max_p.y, (double)v.y);
            min_p.z = std::min(min_p.z, (double)v.z);
            max_p.z = std::max(max_p.z, (double)v.z);
        }
    }
    Vec3 center = (min_p + max_p) * 0.5;
    double extent = std::max({max_p.x - min_p.x, max_p.y - min_p.y, max_p.z - min_p.z});
    double scale = extent > 0.0 ? 1.0 / extent : 1.0;
    out.center = center;
    out.scale = scale;
    out.bounds_min = (min_p - center) * scale;
    out.bounds_max = (max_p - center) * scale;
    for (auto& md : out.meshes) {
        for (auto& v : md.vertices) {
            v.x = static_cast<float>((v.x - center.x) * scale);
            v.y = static_cast<float>((v.y - center.y) * scale);
            v.z = static_cast<float>((v.z - center.z) * scale);
        }
    }

    // ── Skin ─────────────────────────────────────────────────────────────
    // Parse the first skin: joint node list, hierarchy parents, bind local TRS,
    // and inverse bind matrices (IBM). IBMs live in the
    // original model frame, so each is pre-multiplied by the normalization
    // transform M = Scale(S)·Translate(-C) to stay aligned with the normalized
    // vertices (v' = (v - C) * S). node_local stays in the ORIGINAL
    // frame; the animation player rebuilds the normalization M from
    // out.skin.center/scale so the joint matrices land in the normalized frame.
    if (doc.contains("skins") && !doc["skins"].empty()) {
        const auto& sk = doc["skins"][0];
        auto nodes = doc.value("nodes", nlohmann::json::array());
        out.skin.joint_nodes.clear();
        for (const auto& j : sk.value("joints", nlohmann::json::array()))
            out.skin.joint_nodes.push_back(j.get<int>());

        // Build the parent map from each node's children list.
        out.skin.parent.assign(nodes.size(), -1);
        for (size_t i = 0; i < nodes.size(); ++i) {
            for (const auto& ch : nodes[i].value("children", nlohmann::json::array()))
                out.skin.parent[ch.get<int>()] = static_cast<int>(i);
        }

        // Node local TRS (original model frame). glTF rotation is [x,y,z,w].
        out.skin.node_local.resize(nodes.size());
        for (size_t i = 0; i < nodes.size(); ++i) {
            Vec3 t(0, 0, 0);
            if (nodes[i].contains("translation")) {
                auto& tr = nodes[i]["translation"];
                t = Vec3(tr[0].get<double>(), tr[1].get<double>(), tr[2].get<double>());
            }
            Quaternion q = Quaternion::identity();
            if (nodes[i].contains("rotation")) {
                auto& r = nodes[i]["rotation"];  // [x,y,z,w]
                q = Quaternion(r[3].get<double>(), r[0].get<double>(), r[1].get<double>(),
                               r[2].get<double>());
            }
            out.skin.node_local[i] = Mat4::from_position_rotation(t, q);
        }

        // Inverse bind matrices, transformed into the normalized frame.
        int ibm_acc = sk.value("inverseBindMatrices", -1);
        if (ibm_acc >= 0) {
            const auto& a = accessors[ibm_acc];
            int bv = a.value("bufferView", -1);
            size_t off = view_bytes(bv).first + a.value("byteOffset", 0);
            size_t count = a.value("count", 0);
            const float* p = reinterpret_cast<const float*>(buffer.data() + off);
            // Normalization v' = (v - C)·S means IB'_j = Scale(S)·Translate(-C)·IB_j.
            // Translate(-C) puts -C in the translation column; Scale(S) then scales it
            // to -C·S (the correct composition for a column-major matrix).
            float sc = static_cast<float>(out.scale);
            Mat4 T = Mat4::from_position_rotation(-out.center, Quaternion::identity());
            Mat4 M;
            for (int r = 0; r < 3; ++r) {
                M.ptr()[r * 4 + 0] = T.ptr()[r * 4 + 0] * sc;
                M.ptr()[r * 4 + 1] = T.ptr()[r * 4 + 1] * sc;
                M.ptr()[r * 4 + 2] = T.ptr()[r * 4 + 2] * sc;
                M.ptr()[r * 4 + 3] = T.ptr()[r * 4 + 3] * sc;
            }
            M.ptr()[12] = T.ptr()[12] * sc;
            M.ptr()[13] = T.ptr()[13] * sc;
            M.ptr()[14] = T.ptr()[14] * sc;
            M.ptr()[3] = T.ptr()[3];
            M.ptr()[7] = T.ptr()[7];
            M.ptr()[11] = T.ptr()[11];
            M.ptr()[15] = 1.0f;

            out.skin.inverse_bind.clear();
            out.skin.inverse_bind.reserve(count);
            for (size_t i = 0; i < count; ++i) {
                Mat4 ib;
                for (int c = 0; c < 16; ++c) ib.ptr()[c] = p[i * 16 + c];
                out.skin.inverse_bind.push_back(M * ib);
            }
        }
        out.skin.center = out.center;
        out.skin.scale = out.scale;
        out.skin.valid = true;
    }

    // ── Animations ────────────────────────────────────────────────────────
    // Each channel maps to one track: sampler input is a SCALAR times accessor,
    // sampler output is VEC3 (translation/scale) or VEC4 (rotation) values.
    out.animations.clear();
    auto read_times = [&](int acc) {
        int comps = 0;
        return read_accessor_floats(acc, comps);
    };
    for (const auto& an : doc.value("animations", nlohmann::json::array())) {
        GltfAnimation clip;
        clip.name = an.value("name", "");
        auto samplers = an.value("samplers", nlohmann::json::array());
        auto channels = an.value("channels", nlohmann::json::array());
        for (const auto& ch : channels) {
            int s = ch["sampler"].get<int>();
            const auto& target = ch["target"];
            int node = target["node"].get<int>();
            std::string path = target.value("path", "");
            const auto& sam = samplers[s];
            GltfAnimation::Track tr;
            tr.node = node;
            tr.path = path == "rotation" ? GltfAnimation::Rotation
                      : path == "scale"  ? GltfAnimation::Scale
                                         : GltfAnimation::Translation;
            int in_acc = sam["input"].get<int>();
            int out_acc = sam["output"].get<int>();
            tr.times = read_times(in_acc);
            int out_comps = 0;
            tr.values = read_accessor_floats(out_acc, out_comps);
            if (!tr.times.empty()) clip.duration = std::max(clip.duration, tr.times.back());
            clip.tracks.push_back(std::move(tr));
        }
        out.animations.push_back(std::move(clip));
    }

    return !out.meshes.empty();
}

bool load_gltf_file(const std::string& path, GltfModelData& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    return load_gltf_from_memory(ss.str(), out);
}

}  // namespace robcraft::renderer
