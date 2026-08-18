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

#include "robcraft/editor/command/command.hpp"
#include "robcraft/editor/command/entity_snapshot.hpp"
#include "robcraft/engine/world/terrain.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::editor::command {

using namespace robcraft::engine::world;

/** @brief Before/after values of one terrain cell. */
struct TerrainCellDelta {
    /** @brief Cell column index. */
    int cx = 0;
    /** @brief Cell row index. */
    int cz = 0;
    /** @brief Height before / after the edit. */
    float height_before = 0.0f, height_after = 0.0f;
    /** @brief Walkable flag before / after the edit. */
    bool walkable_before = false, walkable_after = false;
    /** @brief Cliff level before / after the edit. */
    uint8_t cliff_before = 0, cliff_after = 0;
    /** @brief Terrain type id before / after the edit. */
    uint8_t type_before = 0, type_after = 0;
    /** @brief Water sentinel before / after (WATER_OFF = dry, else wet). */
    float water_before = Terrain::WATER_OFF, water_after = Terrain::WATER_OFF;
};

/** @brief Restores a captured before/after diff of entities and terrain cells. */
class WorldEditCommand : public ICommand {
public:
    /**
     * @brief Constructs a command from captured state.
     * @param world The world this command edits; must outlive the command.
     * @param label Short label for menus/tooltips.
     * @param before Entity snapshots of the before-state.
     * @param after Entity snapshots of the after-state.
     * @param cells Terrain cell before/after diffs.
     */
    WorldEditCommand(World& world, const char* label,
                     std::vector<std::unique_ptr<EntitySnapshot>> before,
                     std::vector<std::unique_ptr<EntitySnapshot>> after,
                     std::vector<TerrainCellDelta> cells);

    /** @brief Applies the after-state (redo). */
    void execute() override;
    /** @brief Restores the before-state (undo). */
    void undo() override;
    /** @brief Returns the command label. */
    const char* label() const override;

private:
    /**
     * @brief Applies one half of the diff.
     * @param target The snapshot list to restore (before_ or after_).
     * @param use_after True writes after cell values, false writes before.
     */
    void apply(std::vector<std::unique_ptr<EntitySnapshot>>& target, bool use_after) const;

    /** @brief The world this command edits. */
    World* world_ = nullptr;
    /** @brief Command label. */
    const char* label_;
    /** @brief Entity snapshots of the before-state. */
    std::vector<std::unique_ptr<EntitySnapshot>> before_;
    /** @brief Entity snapshots of the after-state. */
    std::vector<std::unique_ptr<EntitySnapshot>> after_;
    /** @brief Terrain cell before/after diffs. */
    std::vector<TerrainCellDelta> cells_;
};

}  // namespace robcraft::editor::command
