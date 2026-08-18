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

#include "robcraft/engine/collision/spatial_grid.hpp"

#include <cmath>

namespace robcraft::engine::collision {

using namespace robcraft::engine::collision;

SpatialGrid::SpatialGrid(double cell_size, double min_x, double min_z, double max_x, double max_z)
    : cell_size_(cell_size),
      origin_(min_x, min_z),
      cols_(static_cast<int>(std::ceil((max_x - min_x) / cell_size))),
      rows_(static_cast<int>(std::ceil((max_z - min_z) / cell_size))) {
    this->cells_.resize(static_cast<size_t>(this->cols_ * this->rows_));
}

void SpatialGrid::insert(Entity e, const Vec3& pos, const BoxCollider& col) {
    this->insert_aabb(e, AABB::from_box(pos, col));
}

void SpatialGrid::insert(Entity e, const Vec3& pos, const BoxCollider& col,
                         const Quaternion& rotation) {
    this->insert_aabb(e, AABB::from_box(pos, col, rotation));
}

void SpatialGrid::insert_aabb(Entity e, const AABB& aabb) {
    int c0 = this->cell_x(aabb.min.x);
    int c1 = this->cell_x(aabb.max.x);
    int r0 = this->cell_z(aabb.min.z);
    int r1 = this->cell_z(aabb.max.z);

    c0 = std::max(0, std::min(c0, this->cols_ - 1));
    c1 = std::max(0, std::min(c1, this->cols_ - 1));
    r0 = std::max(0, std::min(r0, this->rows_ - 1));
    r1 = std::max(0, std::min(r1, this->rows_ - 1));

    for (int r = r0; r <= r1; ++r) {
        for (int c = c0; c <= c1; ++c) {
            this->cells_[r * this->cols_ + c].push_back(e);
        }
    }
}

std::vector<Entity> SpatialGrid::query_ray(const Ray& ray, double max_range) const {
    std::vector<Entity> result;
    std::unordered_set<Entity> seen;

    this->traverse_ray(ray, max_range, [&](int r, int c) {
        if (r < 0 || r >= this->rows_ || c < 0 || c >= this->cols_) return false;
        for (Entity e : this->cells_[r * this->cols_ + c]) {
            if (seen.insert(e).second) {
                result.push_back(e);
            }
        }
        return false;
    });

    return result;
}

std::vector<Entity> SpatialGrid::query_aabb(const AABB& box) const {
    std::vector<Entity> result;
    std::unordered_set<Entity> seen;

    int c0 = this->cell_x(box.min.x);
    int c1 = static_cast<int>(std::ceil((box.max.x - this->origin_.x) / this->cell_size_)) - 1;
    int r0 = this->cell_z(box.min.z);
    int r1 = static_cast<int>(std::ceil((box.max.z - this->origin_.y) / this->cell_size_)) - 1;

    for (int r = r0; r <= r1; ++r) {
        if (r < 0 || r >= this->rows_) continue;
        for (int c = c0; c <= c1; ++c) {
            if (c < 0 || c >= this->cols_) continue;
            for (Entity e : this->cells_[r * this->cols_ + c]) {
                if (seen.insert(e).second) result.push_back(e);
            }
        }
    }
    return result;
}

int SpatialGrid::cell_x(double x) const {
    return static_cast<int>(std::floor((x - this->origin_.x) / this->cell_size_));
}

int SpatialGrid::cell_z(double z) const {
    return static_cast<int>(std::floor((z - this->origin_.y) / this->cell_size_));
}

}  // namespace robcraft::engine::collision
