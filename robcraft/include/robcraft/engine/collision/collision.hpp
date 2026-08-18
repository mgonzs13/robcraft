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

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::collision {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::math;

/** @brief An axis-aligned bounding box in world space. */
struct AABB {
    /** @brief Minimum corner of the box. */
    Vec3 min;
    /** @brief Maximum corner of the box. */
    Vec3 max;

    /**
     * @brief Builds an AABB enclosing an unrotated box collider.
     * @param position Center position of the box.
     * @param box The box collider.
     * @return The enclosing AABB.
     */
    static AABB from_box(const Vec3& position, const BoxCollider& box);
    /**
     * @brief Builds an AABB enclosing a rotated box collider.
     * @param position Center position of the box.
     * @param box The box collider.
     * @param rotation Orientation of the box.
     * @return The enclosing AABB.
     */
    static AABB from_box(const Vec3& position, const BoxCollider& box, const Quaternion& rotation);
    /**
     * @brief Tests whether this box overlaps another.
     * @param other The other box.
     * @return True if the boxes overlap.
     */
    bool overlaps(const AABB& other) const;
    /**
     * @brief Computes the overlap extents against another box.
     * @param other The other box.
     * @return Overlap vector (zero if not overlapping).
     */
    Vec3 penetration(const AABB& other) const;
    /**
     * @brief Computes the minimal translation vector to separate from another box.
     * @param other The other box.
     * @return Separation vector pointing away from the other box.
     */
    Vec3 separation_vector(const AABB& other) const;
    /** @brief Returns the geometric center of the box. */
    Vec3 center() const;
};

/**
 * @brief Resolves an overlap, moving only entities moving into the collision.
 * An entity is corrected only when its world-space velocity has a component
 * toward the other entity; stationary or receding entities are left in place,
 * so parked robots and static objects can never be pushed.
 * @param aabb_a Bounding box of entity A.
 * @param aabb_b Bounding box of entity B.
 * @param vel_a World-space velocity of entity A (zero for static entities).
 * @param vel_b World-space velocity of entity B (zero for static entities).
 * @param pos_a Current position of A; updated when A is moving into the overlap.
 * @param pos_b Current position of B; updated when B is moving into the overlap.
 */
void resolve_overlap(const AABB& aabb_a, const AABB& aabb_b, const Vec3& vel_a, const Vec3& vel_b,
                     Vec3& pos_a, Vec3& pos_b);

}  // namespace robcraft::engine::collision
