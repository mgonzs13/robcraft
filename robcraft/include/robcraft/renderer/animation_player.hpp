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

#include <memory>
#include <string>
#include <vector>

#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/renderer/gltf_loader.hpp"
#include "robcraft/renderer/model.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

/**
 * @brief Samples an animation clip at a time into per-joint matrices
 *        (joint world × inverse bind).
 * @param skin The model skeleton.
 * @param clip The clip to sample.
 * @param t Time in seconds (clamped to [0, duration]).
 * @return Joint matrices in skin.joint_nodes order. */
std::vector<Mat4> sample_animation(const GltfSkin& skin, const GltfAnimation& clip, float t);

/** @brief Plays a skeletal animation clip and produces joint matrices. */
class AnimationPlayer {
public:
    /** @brief Binds a model's skeleton + clips.
     *  @param model The skinned model (may be non-skinned; player stays idle). */
    void set_model(const std::shared_ptr<Model>& model);
    /** @brief Starts playing a clip by name.
     *  @param clip_name Clip name.
     *  @param loop True to loop. */
    void play(const std::string& clip_name, bool loop = true);
    /** @brief Advances the clip by dt and recomputes joint matrices. */
    void update(double dt);
    /** @return Joint matrices for the current time. */
    const std::vector<Mat4>& joint_matrices() const { return this->joint_matrices_; }
    /** @return Name of the currently playing clip, or empty. */
    std::string current_name() const { return this->current_name_; }
    /** @return True if a clip is loaded. */
    bool has_clip() const { return this->clip_ != nullptr; }

private:
    /** @brief The bound skinned model. */
    std::shared_ptr<Model> model_;
    /** @brief Currently playing clip. */
    const GltfAnimation* clip_ = nullptr;
    /** @brief Current time in the clip. */
    double time_ = 0.0;
    /** @brief Loop flag. */
    bool loop_ = true;
    /** @brief Current clip name. */
    std::string current_name_;
    /** @brief Computed joint matrices. */
    std::vector<Mat4> joint_matrices_;
};

}  // namespace robcraft::renderer
