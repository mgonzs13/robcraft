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

#include "robcraft/engine/collision/collision.hpp"

#include <algorithm>
#include <cmath>

namespace robcraft::engine::collision {

using namespace robcraft::engine::collision;

AABB AABB::from_box(const Vec3& position, const BoxCollider& box) {
    return {position - box.half_extents, position + box.half_extents};
}

AABB AABB::from_box(const Vec3& position, const BoxCollider& box, const Quaternion& rotation) {
    // Compute world-space AABB of a rotated box by rotating all 8 corners
    Vec3 he = box.half_extents;
    Vec3 corners[8] = {
        Vec3(-he.x, -he.y, -he.z), Vec3(he.x, -he.y, -he.z), Vec3(-he.x, he.y, -he.z),
        Vec3(he.x, he.y, -he.z),   Vec3(-he.x, -he.y, he.z), Vec3(he.x, -he.y, he.z),
        Vec3(-he.x, he.y, he.z),   Vec3(he.x, he.y, he.z),
    };
    Vec3 mn(1e18, 1e18, 1e18), mx(-1e18, -1e18, -1e18);
    for (int i = 0; i < 8; ++i) {
        Vec3 c = rotation.rotate(corners[i]) + position;
        mn.x = std::min(mn.x, c.x);
        mn.y = std::min(mn.y, c.y);
        mn.z = std::min(mn.z, c.z);
        mx.x = std::max(mx.x, c.x);
        mx.y = std::max(mx.y, c.y);
        mx.z = std::max(mx.z, c.z);
    }
    return {mn, mx};
}

bool AABB::overlaps(const AABB& other) const {
    return (this->min.x <= other.max.x && this->max.x >= other.min.x) &&
           (this->min.y <= other.max.y && this->max.y >= other.min.y) &&
           (this->min.z <= other.max.z && this->max.z >= other.min.z);
}

Vec3 AABB::penetration(const AABB& other) const {
    double dx = std::min(this->max.x - other.min.x, other.max.x - this->min.x);
    double dy = std::min(this->max.y - other.min.y, other.max.y - this->min.y);
    double dz = std::min(this->max.z - other.min.z, other.max.z - this->min.z);
    return {dx, dy, dz};
}

Vec3 AABB::separation_vector(const AABB& other) const {
    Vec3 pen = this->penetration(other);

    if (pen.x <= pen.y && pen.x <= pen.z) {
        double sign = (this->center().x < other.center().x) ? -1.0 : 1.0;
        return {pen.x * sign, 0.0, 0.0};
    }
    if (pen.y <= pen.x && pen.y <= pen.z) {
        double sign = (this->center().y < other.center().y) ? -1.0 : 1.0;
        return {0.0, pen.y * sign, 0.0};
    }
    double sign = (this->center().z < other.center().z) ? -1.0 : 1.0;
    return {0.0, 0.0, pen.z * sign};
}

Vec3 AABB::center() const {
    return (this->min + this->max) * 0.5;
}

void resolve_overlap(const AABB& aabb_a, const AABB& aabb_b, const Vec3& vel_a, const Vec3& vel_b,
                     Vec3& pos_a, Vec3& pos_b) {
    // Separation pushes A away from B; negate it to move B away from A.
    Vec3 sep = aabb_a.separation_vector(aabb_b);

    // An entity encroaches when it moves toward the other: A along -sep, B along +sep.
    bool a_encroach = vel_a.dot(sep) < 0.0;
    bool b_encroach = vel_b.dot(sep) > 0.0;

    if (a_encroach && b_encroach) {
        pos_a += sep * 0.5;
        pos_b -= sep * 0.5;
    } else if (a_encroach) {
        pos_a += sep;
    } else if (b_encroach) {
        pos_b -= sep;
    }
}

}  // namespace robcraft::engine::collision
