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

#include "robcraft/renderer/animation_player.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

namespace {

/**
 * @brief Linearly interpolates two 3-float values.
 * @param a Start value (3 floats).
 * @param b End value (3 floats).
 * @param t Blend factor in [0, 1].
 * @return The interpolated vector.
 */
Vec3 lerp3(const float* a, const float* b, float t) {
    return Vec3(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t);
}

/**
 * @brief Normalized linear interpolation of two unit quaternions (shortest path).
 * @param a Start quaternion.
 * @param b End quaternion.
 * @param t Blend factor in [0, 1].
 * @return The interpolated unit quaternion.
 */
Quaternion nlerp(const Quaternion& a, const Quaternion& b, float t) {
    double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    Quaternion q = dot < 0.0 ? Quaternion(-b.w, -b.x, -b.y, -b.z) : b;
    Quaternion r(a.w + (q.w - a.w) * t, a.x + (q.x - a.x) * t, a.y + (q.y - a.y) * t,
                 a.z + (q.z - a.z) * t);
    double len = std::sqrt(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
    return len > 0.0 ? Quaternion(r.w / len, r.x / len, r.y / len, r.z / len)
                     : Quaternion::identity();
}

/**
 * @brief Evaluates one track at time t into the matching TRS output.
 * @param tr The track (translation/rotation/scale keyframes).
 * @param t Time (clamped to the track's keyframe range).
 * @param out_t Translation output.
 * @param out_q Rotation output (glTF [x,y,z,w] → rc (w,x,y,z)).
 * @param out_s Scale output.
 */
void eval_track(const GltfAnimation::Track& tr, float t, Vec3& out_t, Quaternion& out_q,
                Vec3& out_s) {
    if (tr.times.empty()) return;
    float t0 = tr.times.front();
    float t1 = tr.times.back();
    float ct = std::max(t0, std::min(t1, t));
    size_t i = 0;
    while (i + 1 < tr.times.size() && tr.times[i + 1] < ct) ++i;
    float a = tr.times[i];
    float b = (i + 1 < tr.times.size()) ? tr.times[i + 1] : a;
    float f = (b > a) ? (ct - a) / (b - a) : 0.0f;
    size_t comps = tr.path == GltfAnimation::Rotation ? 4 : 3;
    const float* va = &tr.values[i * comps];
    const float* vb = (i + 1 < tr.times.size()) ? &tr.values[(i + 1) * comps] : va;
    if (tr.path == GltfAnimation::Rotation) {
        Quaternion qa(va[3], va[0], va[1], va[2]);
        Quaternion qb(vb[3], vb[0], vb[1], vb[2]);
        out_q = nlerp(qa, qb, f);
    } else if (tr.path == GltfAnimation::Translation) {
        out_t = lerp3(va, vb, f);
    } else {
        out_s = lerp3(va, vb, f);
    }
}

}  // namespace

std::vector<Mat4> sample_animation(const GltfSkin& skin, const GltfAnimation& clip, float t) {
    const size_t n = skin.node_local.size();
    if (n == 0) return {};

    // Frame consistency: vertices are normalized (v' = (v - C)·S) while the skin's
    // node_local and animation keyframes stay in the ORIGINAL model
    // frame. The joint matrix that maps a normalized bind vertex onto its animated
    // position is therefore the conjugation M·(world·IBM_raw)·M⁻¹, where
    // M = Scale(S)·Translate(-C) and IBM_raw = M⁻¹·inverse_bind (the loader stores
    // inverse_bind pre-multiplied by M).
    double s = skin.scale > 0.0 ? skin.scale : 1.0;
    Mat4 M = Mat4::scale_matrix(Vec3(s, s, s)) *
             Mat4::from_position_rotation(-skin.center, Quaternion::identity());
    Mat4 Minv = Mat4::from_position_rotation(skin.center, Quaternion::identity()) *
                Mat4::scale_matrix(Vec3(1.0 / s, 1.0 / s, 1.0 / s));

    std::vector<Vec3> trans(n, Vec3(0, 0, 0));
    std::vector<Quaternion> rots(n, Quaternion::identity());
    std::vector<Vec3> scales(n, Vec3(1, 1, 1));
    std::vector<char> tracked(n, 0);
    for (const auto& tr : clip.tracks) {
        if (tr.node < 0 || tr.node >= static_cast<int>(n)) continue;
        tracked[tr.node] = 1;
        eval_track(tr, t, trans[tr.node], rots[tr.node], scales[tr.node]);
    }

    // Per-node local matrices: animated TRS for tracked nodes, bind pose for the rest.
    std::vector<Mat4> local(n);
    for (size_t i = 0; i < n; ++i) {
        if (tracked[i])
            local[i] = Mat4::scale_matrix(Vec3(scales[i].x, scales[i].y, scales[i].z)) *
                       Mat4::from_position_rotation(trans[i], rots[i]);
        else
            local[i] = skin.node_local[i];
    }

    // World matrices via the full ancestor chain (glTF nodes are NOT topologically
    // ordered — the mech has parents with index > child, so a single forward pass
    // would collapse the hierarchy).
    std::vector<Mat4> world(n);
    for (size_t i = 0; i < n; ++i) {
        std::vector<int> chain;
        for (int cur = static_cast<int>(i); cur >= 0 && chain.size() < n; cur = skin.parent[cur])
            chain.push_back(cur);
        Mat4 w = local[chain.back()];
        for (int k = static_cast<int>(chain.size()) - 2; k >= 0; --k) w = w * local[chain[k]];
        world[i] = w;
    }

    std::vector<Mat4> joints(skin.joint_nodes.size());
    for (size_t j = 0; j < skin.joint_nodes.size(); ++j) {
        int node = skin.joint_nodes[j];
        if (node < 0 || node >= static_cast<int>(n)) {
            joints[j] = Mat4();
            continue;
        }
        Mat4 ib = j < skin.inverse_bind.size() ? skin.inverse_bind[j] : Mat4();
        joints[j] = M * (world[node] * (Minv * ib)) * Minv;
    }
    return joints;
}

void AnimationPlayer::set_model(const std::shared_ptr<Model>& model) {
    this->model_ = model;
    this->clip_ = nullptr;
    this->current_name_.clear();
    this->time_ = 0.0;
    this->loop_ = true;
    this->joint_matrices_.clear();
    if (model && model->skinned() && !model->animations().empty()) {
        this->clip_ = &model->animations().front();
        this->current_name_ = this->clip_->name;
    }
}

void AnimationPlayer::play(const std::string& clip_name, bool loop) {
    this->loop_ = loop;
    if (!this->model_) return;
    for (const auto& a : this->model_->animations()) {
        if (a.name == clip_name) {
            this->clip_ = &a;
            this->current_name_ = a.name;
            this->time_ = 0.0;
            return;
        }
    }
    // Unknown clip: fall back to the first available clip.
    if (!this->model_->animations().empty()) {
        this->clip_ = &this->model_->animations().front();
        this->current_name_ = this->clip_->name;
        this->time_ = 0.0;
    }
}

void AnimationPlayer::update(double dt) {
    if (!this->model_ || !this->clip_ || !this->model_->skinned()) return;
    this->time_ += dt;
    if (this->loop_ && this->clip_->duration > 0.0f)
        this->time_ = std::fmod(this->time_, static_cast<double>(this->clip_->duration));
    this->joint_matrices_ =
        sample_animation(this->model_->skin_ref(), *this->clip_, static_cast<float>(this->time_));
}

}  // namespace robcraft::renderer
