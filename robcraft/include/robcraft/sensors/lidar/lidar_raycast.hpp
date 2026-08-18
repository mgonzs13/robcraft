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
#include "robcraft/engine/collision/spatial_grid.hpp"
#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/ecs/component_store.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::sensors::lidar {

using namespace robcraft::engine::world;
using namespace robcraft::engine::collision;
using namespace robcraft::engine::core;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::math;

/** @brief Builds the broad-phase grid once and queries it per ray.
 *  @note Populating the grid is O(colliders); querying is O(candidates). The
 *        sensors query many rays per update, so the grid is built once here. */
class LidarScene {
public:
    /** @brief Builds the broad-phase grid over all colliders except self_entity.
     *  @param world The world being sensed.
     *  @param self_entity The sensor's own entity (excluded from hits).
     *  @param grid_cell Grid cell size for the broad-phase.
     *  @param grid_half_x Half-width of the grid footprint.
     *  @param grid_half_z Half-depth of the grid footprint.
     *  @param use_rotation When true, insert rotated collider AABBs (matches the
     *         3D sensor's original broad-phase); when false, use axis-aligned
     *         footprints (matches the 2D sensor's original broad-phase). */
    LidarScene(const World& world, Entity self_entity, double grid_cell, double grid_half_x,
               double grid_half_z, bool use_rotation);

    /** @brief Returns the nearest hit distance for a ray.
     *  @param sensor_pos World-space ray origin.
     *  @param dir World-space ray direction.
     *  @param range_min Minimum distance to accept.
     *  @param range_max Maximum distance to test; hits beyond it are ignored.
     *  @param include_terrain When true, also test the terrain heightmap.
     *  @return The closest hit distance in [range_min, range_max], or +inf when
     *          nothing is hit within range_max (no detection). */
    double raycast_hit(const Vec3& sensor_pos, const Vec3& dir, double range_min, double range_max,
                       bool include_terrain) const;

private:
    /** @brief The world being sensed. */
    const World* world_;
    /** @brief Transform store cached at construction. */
    const ComponentStore<Transform3D>* tf_store_;
    /** @brief Collider store cached at construction. */
    const ComponentStore<BoxCollider>* col_store_;
    /** @brief Broad-phase grid populated at construction. */
    SpatialGrid grid_;
};

}  // namespace robcraft::sensors::lidar
