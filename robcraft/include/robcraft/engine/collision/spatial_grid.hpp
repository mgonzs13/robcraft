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

#include <limits>
#include <unordered_set>
#include <vector>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/math/vec2.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::collision {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

/** @brief Uniform spatial grid storing entities by cell for broad-phase queries. */
class SpatialGrid {
public:
    /**
     * @brief Constructs a grid over the given world-space bounds.
     * @param cell_size Edge length of each square cell.
     * @param min_x Minimum x world coordinate.
     * @param min_z Minimum z world coordinate.
     * @param max_x Maximum x world coordinate.
     * @param max_z Maximum z world coordinate.
     */
    SpatialGrid(double cell_size, double min_x, double min_z, double max_x, double max_z);

    /**
     * @brief Inserts an entity into all cells overlapped by its box collider.
     * @param e The entity to insert.
     * @param pos Center position of the collider.
     * @param col The collider used to compute the AABB footprint.
     */
    void insert(Entity e, const Vec3& pos, const BoxCollider& col);
    /**
     * @brief Inserts an entity using its rotated box collider footprint.
     * @param e The entity to insert.
     * @param pos Center position of the collider.
     * @param col The collider used to compute the AABB footprint.
     * @param rotation Orientation of the collider.
     */
    void insert(Entity e, const Vec3& pos, const BoxCollider& col, const Quaternion& rotation);
    /**
     * @brief Returns all distinct entities whose cells the ray crosses.
     * @param ray The ray to trace.
     * @param max_range Maximum distance to trace.
     * @return Unique entities hit along the ray, in traversal order.
     */
    std::vector<Entity> query_ray(const Ray& ray, double max_range) const;
    /**
     * @brief Returns all distinct entities whose cells overlap an AABB.
     * @param box The query box in world space.
     * @return Unique entities overlapping the box. The box's max edges are
     * exclusive (half-open [min, max)).
     */
    std::vector<Entity> query_aabb(const AABB& box) const;

private:
    /** @brief Column index for a world-space x coordinate. */
    int cell_x(double x) const;
    /** @brief Row index for a world-space z coordinate. */
    int cell_z(double z) const;

    /**
     * @brief Inserts an entity into every cell overlapped by an AABB.
     * @param e The entity to insert.
     * @param aabb The footprint to insert into.
     */
    void insert_aabb(Entity e, const AABB& aabb);

    /**
     * @brief Walks the cells crossed by a ray, invoking a callback per cell.
     * @param ray The ray to trace.
     * @param max_range Maximum distance to trace.
     * @param callback Invoked as callback(row, col); return true to stop early.
     */
    template <typename F>
    void traverse_ray(const Ray& ray, double max_range, F&& callback) const {
        double ox = ray.origin.x;
        double oz = ray.origin.z;
        double dx = ray.direction.x;
        double dz = ray.direction.z;

        int cx = this->cell_x(ox);
        int cz = this->cell_z(oz);

        if (cx < 0 || cx >= this->cols_ || cz < 0 || cz >= this->rows_) return;

        int step_x = (dx > 0) ? 1 : (dx < 0) ? -1 : 0;
        int step_z = (dz > 0) ? 1 : (dz < 0) ? -1 : 0;

        double t_delta_x = (std::abs(dx) > 1e-12) ? std::abs(this->cell_size_ / dx)
                                                  : std::numeric_limits<double>::infinity();
        double t_delta_z = (std::abs(dz) > 1e-12) ? std::abs(this->cell_size_ / dz)
                                                  : std::numeric_limits<double>::infinity();

        double next_x = (dx > 0) ? (this->origin_.x + (cx + 1) * this->cell_size_)
                                 : (this->origin_.x + cx * this->cell_size_);
        double t_max_x =
            (std::abs(dx) > 1e-12) ? (next_x - ox) / dx : std::numeric_limits<double>::infinity();

        double next_z = (dz > 0) ? (this->origin_.y + (cz + 1) * this->cell_size_)
                                 : (this->origin_.y + cz * this->cell_size_);
        double t_max_z =
            (std::abs(dz) > 1e-12) ? (next_z - oz) / dz : std::numeric_limits<double>::infinity();

        double t = 0.0;

        while (t < max_range) {
            if (callback(cz, cx)) break;

            if (t_max_x < t_max_z) {
                t = t_max_x;
                if (t > max_range) break;
                cx += step_x;
                if (cx < 0 || cx >= this->cols_) break;
                t_max_x += t_delta_x;
            } else {
                t = t_max_z;
                if (t > max_range) break;
                cz += step_z;
                if (cz < 0 || cz >= this->rows_) break;
                t_max_z += t_delta_z;
            }
        }
    }

    /** @brief Edge length of each cell. */
    double cell_size_;
    /** @brief World-space origin (minimum corner) of the grid. */
    Vec2 origin_;
    /** @brief Number of grid columns. */
    int cols_;
    /** @brief Number of grid rows. */
    int rows_;
    /** @brief Cell storage indexed as [row * cols_ + col]. */
    std::vector<std::vector<Entity>> cells_;
};

}  // namespace robcraft::engine::collision
