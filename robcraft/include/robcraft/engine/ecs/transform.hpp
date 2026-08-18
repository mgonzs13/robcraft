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

#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::ecs {

using namespace robcraft::engine::ecs;
using namespace robcraft::engine::math;

/** @brief Full 3D transform for objects in world space. */
struct Transform3D {
    /** @brief Position in world space. */
    Vec3 position;
    /** @brief Rotation in world space. */
    Quaternion rotation = Quaternion::identity();
    /** @brief Per-axis scale. */
    Vec3 scale{1.0, 1.0, 1.0};

    Transform3D() = default;

    /**
     * @brief Constructs a transform at a position with identity rotation.
     * @param pos Position in world space.
     */
    Transform3D(const Vec3& pos) : position(pos) {}

    /**
     * @brief Constructs a transform from a position and rotation.
     * @param pos Position in world space.
     * @param rot Rotation in world space.
     */
    Transform3D(const Vec3& pos, const Quaternion& rot) : position(pos), rotation(rot) {}
};

}  // namespace robcraft::engine::ecs
