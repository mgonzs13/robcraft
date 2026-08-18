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

#include <memory>
#include <vector>

#include "robcraft/editor/command/world_edit_command.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::editor {

using namespace robcraft::editor::command;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;

/** @brief Accumulates before/after state of one editor gesture. */
class EditRecorder {
public:
    /**
     * @brief Constructs a recorder bound to a world.
     * @param world The world being edited.
     */
    explicit EditRecorder(World& world);

    /** @brief Resets accumulated state, starting a new capture. */
    void begin();
    /** @brief Captures an entity's current state as part of the before-state.
     *  @param e The entity. */
    void record_before_entity(Entity e);
    /** @brief Captures a cell's current state as part of the before-state.
     *  @param cx Column index.
     *  @param cz Row index. */
    void record_before_cell(int cx, int cz);
    /** @brief Captures the cells under an entity's collider as before-state.
     *  @param e The entity. */
    void record_before_cells_under(Entity e);
    /** @brief Captures an entity's current state as part of the after-state.
     *  @param e The entity. */
    void record_after_entity(Entity e);
    /** @brief Builds the command, refreshing after-cell values from the world.
     *  @param label Short label for the command.
     *  @return The command, or nullptr if nothing was captured. */
    std::unique_ptr<WorldEditCommand> finish(const char* label);

private:
    /** @brief Index of a recorded cell, or -1 if absent.
     *  @param cx Column index.
     *  @param cz Row index.
     *  @return The cell index, or -1. */
    int find_cell(int cx, int cz) const;

    /** @brief The world being edited. */
    World& world_;
    /** @brief Before-state entity snapshots. */
    std::vector<std::unique_ptr<EntitySnapshot>> before_;
    /** @brief After-state entity snapshots. */
    std::vector<std::unique_ptr<EntitySnapshot>> after_;
    /** @brief Terrain cell before/after diffs. */
    std::vector<TerrainCellDelta> cells_;
};

}  // namespace robcraft::editor
