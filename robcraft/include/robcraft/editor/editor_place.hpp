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
#include <vector>

#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/math/cell_range.hpp"
#include "robcraft/engine/world/terrain.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::core;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

class EditorApp;
enum class PlaceableType;

/** @brief Handles placeable placement, entity picking, and multi-selection. */
class EditorPlacement {
public:
    /** @brief Constructs the placement tools.
     *  @param app The owning editor application. */
    explicit EditorPlacement(EditorApp& app);

    /** @brief Picks the entity under the cursor (colliders first, then doodads).
     *  @return The hit entity, or INVALID_ENTITY. */
    robcraft::engine::core::Entity pick_entity() const;
    /** @brief Handles a Select-tool click with Ctrl for multi-selection. */
    void handle_select_click();
    /** @brief Returns the pick ray under the cursor.
     *  @return The cursor ray. */
    robcraft::engine::collision::Ray cursor_ray() const;
    /** @brief Computes the terrain cell under the cursor.
     *  @param cx Out-param receiving the column.
     *  @param cz Out-param receiving the row.
     *  @return True if the cursor is over terrain. */
    bool cursor_cell(int& cx, int& cz) const;
    /** @brief Starts a wall/floor drag or places a single placeable on mouse press. */
    void handle_place_press();
    /** @brief Finalizes a wall/floor drag on mouse release. */
    void handle_place_release();
    /** @brief Places a single non-drag placeable snapped to a cell. */
    void place_single(int cx, int cz);
    /** @brief Places a wall run and marks its cells non-walkable. */
    void place_wall_run(int ax, int az, int bx, int bz);
    /** @brief Finds endpoint cells of a run that join a perpendicular existing wall.
     *  @param run The run to test.
     *  @return One 1-cell range per corner join (deduplicated for 1-cell runs). */
    std::vector<robcraft::engine::math::CellRange> find_corner_joins(
        const robcraft::engine::math::CellRange& run) const;
    /** @brief Whether a wall corner pillar already covers the cell center.
     *  @param cx Column index.
     *  @param cz Row index.
     *  @return True if a wall-named square-collider pillar covers the cell. */
    bool cell_has_pillar(int cx, int cz) const;
    /** @brief Whether a wall run can merge with a given wall entity.
     *  @param run The new run.
     *  @param e The wall entity to test.
     *  @param out_union Receives the union range when mergeable.
     *  @return True if the entity is a parallel run that overlaps or abuts the run. */
    bool wall_mergeable(const robcraft::engine::math::CellRange& run,
                        robcraft::engine::core::Entity e,
                        robcraft::engine::math::CellRange& out_union) const;
    /** @brief Places a floor rectangle (walkable surface). */
    void place_floor_rect(int ax, int az, int bx, int bz);
    /** @brief Fills a transform for a wall run ghost/entity.
     *  @param terrain The terrain to compute heights from.
     *  @param run The cell range of the run.
     *  @param tf Out-param receiving position and scale. */
    void fill_wall_run_transform(const robcraft::engine::world::Terrain& terrain,
                                 const robcraft::engine::math::CellRange& run,
                                 robcraft::engine::ecs::Transform3D& tf) const;
    /** @brief Fills a transform for a floor rectangle ghost/entity.
     *  @param terrain The terrain to compute heights from.
     *  @param rect The cell range of the rectangle.
     *  @param tf Out-param receiving position and scale. */
    void fill_floor_rect_transform(const robcraft::engine::world::Terrain& terrain,
                                   const robcraft::engine::math::CellRange& rect,
                                   robcraft::engine::ecs::Transform3D& tf) const;
    /** @brief Returns the name prefix a placed entity of this type should get.
     *  @param type The placeable type.
     *  @return Prefix string (e.g. "pine_1" for PlaceableType::Pine1). */
    const char* prefix_for_type(PlaceableType type) const;
    /** @brief Deletes all selected entities and restores walkability. */
    void delete_selection();
    /** @brief Restores walkability for cells under an entity's collider. */
    void restore_walkable_under(robcraft::engine::core::Entity e);
    /** @brief Returns the primary (last-clicked) selected entity. */
    robcraft::engine::core::Entity primary_selection() const;
    /** @brief Checks whether an entity is in the selection. */
    bool is_selected(robcraft::engine::core::Entity e) const;
    /** @brief Clears the current selection. */
    void clear_selection();
    /** @brief Adds an entity to the selection if not already present. */
    void add_selection(robcraft::engine::core::Entity e);
    /** @brief Toggles an entity in the selection. */
    void toggle_selection(robcraft::engine::core::Entity e);
    /** @brief Places a robot at the given world position with the given mech prefix.
     *  @param wx World x.
     *  @param wz World z.
     *  @param prefix Mech name prefix ("robot_mike", "robot_leela", ...). */
    void handle_robot_placement(double wx, double wz, const char* prefix);

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
