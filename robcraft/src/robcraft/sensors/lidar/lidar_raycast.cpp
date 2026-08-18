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

#include "robcraft/sensors/lidar/lidar_raycast.hpp"

#include <cmath>
#include <limits>

#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/world/world.hpp"

namespace robcraft::sensors::lidar {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::world;

LidarScene::LidarScene(const World& world, Entity self_entity, double grid_cell, double grid_half_x,
                       double grid_half_z, bool use_rotation)
    : world_(&world),
      tf_store_(world.store<Transform3D>()),
      col_store_(world.store<BoxCollider>()),
      grid_(grid_cell, -grid_half_x, -grid_half_z, grid_half_x, grid_half_z) {
    if (!this->tf_store_ || !this->col_store_) return;
    for (auto& [entity, col] : *this->col_store_) {
        if (entity == self_entity) continue;
        auto* tf = this->tf_store_->get(entity);
        if (!tf) continue;
        if (use_rotation) {
            this->grid_.insert(entity, tf->position, col, tf->rotation);
        } else {
            this->grid_.insert(entity, tf->position, col);
        }
    }
}

double LidarScene::raycast_hit(const Vec3& sensor_pos, const Vec3& dir, double range_min,
                               double range_max, bool include_terrain) const {
    Ray ray{sensor_pos, dir};
    double closest = std::numeric_limits<double>::infinity();
    if (this->tf_store_ && this->col_store_) {
        for (Entity other : this->grid_.query_ray(ray, range_max)) {
            auto* tf = this->tf_store_->get(other);
            auto* col = this->col_store_->get(other);
            if (!tf || !col) continue;
            auto aabb = AABB::from_box(tf->position, *col, tf->rotation);
            auto hit = ray_aabb_intersection(ray, aabb);
            if (hit.has_value() && *hit < closest && *hit >= range_min && *hit <= range_max) {
                closest = *hit;
            }
        }
    }
    if (include_terrain && this->world_->has_terrain()) {
        auto thit = this->world_->terrain().raycast(ray);
        if (thit.has_value() && *thit < closest && *thit >= range_min && *thit <= range_max) {
            closest = *thit;
        }
    }
    return closest;
}

}  // namespace robcraft::sensors::lidar
