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

#include "robcraft/engine/collision/raycast.hpp"

#include <algorithm>
#include <limits>

#include "robcraft/engine/collision/collision.hpp"

namespace robcraft::engine::collision {

using namespace robcraft::engine::collision;

std::optional<double> ray_aabb_intersection(const Ray& ray, const AABB& box) {
    double tmin = -std::numeric_limits<double>::infinity();
    double tmax = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 3; ++i) {
        double origin = (i == 0) ? ray.origin.x : (i == 1) ? ray.origin.y : ray.origin.z;
        double dir = (i == 0) ? ray.direction.x : (i == 1) ? ray.direction.y : ray.direction.z;
        double bmin = (i == 0) ? box.min.x : (i == 1) ? box.min.y : box.min.z;
        double bmax = (i == 0) ? box.max.x : (i == 1) ? box.max.y : box.max.z;

        if (std::abs(dir) < 1e-12) {
            if (origin < bmin || origin > bmax) {
                return std::nullopt;
            }
        } else {
            double inv = 1.0 / dir;
            double t1 = (bmin - origin) * inv;
            double t2 = (bmax - origin) * inv;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
        }
    }

    if (tmin > tmax || tmax < 0.0) {
        return std::nullopt;
    }

    return tmin >= 0.0 ? tmin : 0.0;
}

}  // namespace robcraft::engine::collision
