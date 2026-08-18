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

#include <string>
#include <utility>

#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::world {
class World;
}  // namespace robcraft::engine::world

namespace robcraft::renderer {

using namespace robcraft::engine::math;

class Model;
class ModelCache;

/** @brief Returns the OBJ path for a resolved prefix.
 *  @param prefix A placeable name prefix (e.g. "tree").
 *  @return The OBJ path, or the default box path. */
std::string model_path_for_prefix(const std::string& prefix);

/** @brief Returns the .world mesh label for an entity name.
 *  @param name Entity Name component value.
 *  @return Mesh label (e.g. "wall", "pine", "tree_2"), or "cube" for
 *          names with no known model/primitive prefix. */
std::string mesh_label_for_name(const std::string& name);

/** @brief How a placeable object is sized and grounded. */
struct PlacementSpec {
    /** @brief Entity name prefix (e.g. "tree"); longest match wins. */
    std::string name_prefix;
    /** @brief Model file path; empty = procedural primitive. */
    std::string model_path;
    /** @brief Base world scale at scale factor 1 (unit-normalized model × this). */
    Vec3 base_scale;
    /** @brief Legacy fraction of base_scale.y sitting above the ground (0.5 = centered).
     *  Ignored when the model loads and real bounds are available. */
    float ground_frac = 0.5f;
    /** @brief True when the object may span multiple terrain cells. */
    bool multi_tile = false;
    /** @brief True when the object blocks movement (BoxCollider + non-walkable cells). */
    bool solid = false;
};

/** @brief Returns the placement spec whose prefix best matches an entity name.
 *  @param name Entity Name component value.
 *  @return Matching spec, or nullptr. */
const PlacementSpec* placement_spec_for_name(const std::string& name);

/** @brief Returns the placement spec for an exact prefix (or nullptr).
 *  @param prefix A placeable name prefix (e.g. "tree"). */
const PlacementSpec* placement_spec_for_prefix(const std::string& prefix);

/** @brief Resolves an entity name to a drawable model path, preferring the
 *  PlacementSpec when one matches.
 *  @param name Entity Name component value.
 *  @return Model path, or empty for a procedural primitive (wall/floor). */
std::string draw_model_path_for_name(const std::string& name);

/** @brief Computes the height offset above the ground for a placement.
 *  Uses the model's real normalized bounds when available; otherwise falls
 *  back to the spec's ground_frac (procedural primitives).
 *  @param spec Placement spec (may be null).
 *  @param model Loaded model (may be null/empty when procedural).
 *  @param scale World scale the model is drawn at.
 *  @return Y offset to add to the ground height so the object sits on it. */
float placement_ground_offset(const PlacementSpec* spec, const Model* model, const Vec3& scale);

/** @brief Returns the number of terrain cells an object's footprint covers.
 *  @param spec Placement spec (may be null).
 *  @param cell_size World-space size of one terrain cell.
 *  @return (width, depth) in cells along x and z. */
std::pair<int, int> placement_footprint_cells(const PlacementSpec* spec, double cell_size);

/** @brief Computes world-space box collider half extents for a drawn model.
 *  Uses the model's real unit-normalized bounds scaled to the world scale;
 *  falls back to a cube of `scale` when no model is available.
 *  @param model The loaded model (may be null/empty when procedural).
 *  @param scale World scale the model is drawn at.
 *  @return Half extents per axis. */
Vec3 collider_half_extents(const Model* model, const Vec3& scale);

/** @brief Refits every entity's BoxCollider to its drawn model's real bounds.
 *  Entities whose name resolves to a model (via `draw_model_path_for_name`)
 *  get their collider recomputed from the model's normalized bounds and the
 *  entity's transform scale; procedural entities (walls, floors) are skipped.
 *  @param world The world whose colliders are refitted.
 *  @param cache Model cache used to load models. */
void refit_world_colliders(robcraft::engine::world::World& world, ModelCache& cache);

}  // namespace robcraft::renderer
