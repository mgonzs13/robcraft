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

#include "robcraft/editor/command/world_edit_command.hpp"

#include <unordered_set>

#include "robcraft/engine/world/world.hpp"

namespace robcraft::editor::command {

using namespace robcraft::engine::world;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;

WorldEditCommand::WorldEditCommand(World& world, const char* label,
                                   std::vector<std::unique_ptr<EntitySnapshot>> before,
                                   std::vector<std::unique_ptr<EntitySnapshot>> after,
                                   std::vector<TerrainCellDelta> cells)
    : world_(&world),
      label_(label),
      before_(std::move(before)),
      after_(std::move(after)),
      cells_(std::move(cells)) {}

void WorldEditCommand::execute() {
    this->apply(this->after_, true);
}

void WorldEditCommand::undo() {
    this->apply(this->before_, false);
}

const char* WorldEditCommand::label() const {
    return this->label_;
}

void WorldEditCommand::apply(std::vector<std::unique_ptr<EntitySnapshot>>& target,
                             bool use_after) const {
    World& world = *this->world_;

    // Which entity ids must exist after this apply.
    std::unordered_set<Entity> target_ids;
    for (const auto& snap : target) target_ids.insert(snap->entity());

    // Destroy every alive entity this command references that is not in target.
    auto destroy_others = [&](const std::vector<std::unique_ptr<EntitySnapshot>>& list) {
        for (const auto& snap : list) {
            Entity id = snap->entity();
            if (id == INVALID_ENTITY) continue;
            if (target_ids.count(id)) continue;
            if (world.valid(id)) world.destroy_entity(id);
        }
    };
    destroy_others(this->before_);
    destroy_others(this->after_);

    // Restore target snapshots (rewrite in place or recreate).
    for (auto& snap : target) snap->restore(world);

    // Apply terrain cell deltas.
    if (world.has_terrain()) {
        Terrain& t = world.terrain();
        for (const TerrainCellDelta& c : this->cells_) {
            if (!t.in_bounds(c.cx, c.cz)) continue;
            if (use_after) {
                t.set_height(c.cx, c.cz, c.height_after);
                t.set_walkable(c.cx, c.cz, c.walkable_after);
                t.set_cliff_level(c.cx, c.cz, c.cliff_after);
                t.set_terrain_type(c.cx, c.cz, static_cast<TerrainType>(c.type_after));
                t.set_water(c.cx, c.cz, c.water_after != Terrain::WATER_OFF);
            } else {
                t.set_height(c.cx, c.cz, c.height_before);
                t.set_walkable(c.cx, c.cz, c.walkable_before);
                t.set_cliff_level(c.cx, c.cz, c.cliff_before);
                t.set_terrain_type(c.cx, c.cz, static_cast<TerrainType>(c.type_before));
                t.set_water(c.cx, c.cz, c.water_before != Terrain::WATER_OFF);
            }
        }
    }
}

}  // namespace robcraft::editor::command
