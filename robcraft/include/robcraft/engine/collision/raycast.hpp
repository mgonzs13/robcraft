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

#include <optional>

#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::collision {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::math;

/** @brief A ray defined by an origin and a direction. */
struct Ray {
    /** @brief Starting point of the ray. */
    Vec3 origin;
    /** @brief Direction of the ray (need not be normalized). */
    Vec3 direction;
};

struct AABB;

/**
 * @brief Computes the intersection distance between a ray and an AABB.
 * @param ray The ray.
 * @param box The box to test against.
 * @return Distance along the ray to the first hit, or nullopt if no hit.
 */
std::optional<double> ray_aabb_intersection(const Ray& ray, const AABB& box);

}  // namespace robcraft::engine::collision
