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

#include "robcraft/editor/editor_undo.hpp"

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/world.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::robots::differential_drive;

EditRecorder::EditRecorder(World& world) : world_(world) {}

void EditRecorder::begin() {
    this->before_.clear();
    this->after_.clear();
    this->cells_.clear();
}

void EditRecorder::record_before_entity(Entity e) {
    auto snap = EntitySnapshot::capture(this->world_, e);
    if (snap) this->before_.push_back(std::move(snap));
}

void EditRecorder::record_before_cell(int cx, int cz) {
    if (this->find_cell(cx, cz) >= 0) return;
    if (!this->world_.has_terrain()) return;
    Terrain& t = this->world_.terrain();
    if (!t.in_bounds(cx, cz)) return;
    TerrainCellDelta cell;
    cell.cx = cx;
    cell.cz = cz;
    cell.height_before = cell.height_after = t.height_at(cx, cz);
    cell.walkable_before = cell.walkable_after = t.is_walkable(cx, cz);
    cell.cliff_before = cell.cliff_after = t.cliff_level(cx, cz);
    cell.type_before = cell.type_after = static_cast<uint8_t>(t.terrain_type(cx, cz));
    cell.water_before = cell.water_after = t.has_water(cx, cz) ? 1.0f : Terrain::WATER_OFF;
    this->cells_.push_back(cell);
}

void EditRecorder::record_before_cells_under(Entity e) {
    auto* tf = this->world_.get_component<Transform3D>(e);
    auto* col = this->world_.get_component<BoxCollider>(e);
    if (!tf || !col) return;
    if (!this->world_.has_terrain()) return;
    Terrain& t = this->world_.terrain();
    const double eps = 1e-6;
    int x0, z0, x1, z1;
    t.world_to_cell(tf->position.x - col->half_extents.x, tf->position.z - col->half_extents.z, x0,
                    z0);
    t.world_to_cell(tf->position.x + col->half_extents.x - eps,
                    tf->position.z + col->half_extents.z - eps, x1, z1);
    for (int z = z0; z <= z1; ++z)
        for (int x = x0; x <= x1; ++x)
            if (t.in_bounds(x, z)) this->record_before_cell(x, z);
}

void EditRecorder::record_after_entity(Entity e) {
    auto snap = EntitySnapshot::capture(this->world_, e);
    if (snap) this->after_.push_back(std::move(snap));
}

std::unique_ptr<WorldEditCommand> EditRecorder::finish(const char* label) {
    if (this->before_.empty() && this->after_.empty() && this->cells_.empty()) {
        return nullptr;
    }
    // Refresh after-values from the current terrain (brush strokes mutated live).
    if (this->world_.has_terrain()) {
        Terrain& t = this->world_.terrain();
        for (TerrainCellDelta& cell : this->cells_) {
            cell.height_after = t.height_at(cell.cx, cell.cz);
            cell.walkable_after = t.is_walkable(cell.cx, cell.cz);
            cell.cliff_after = t.cliff_level(cell.cx, cell.cz);
            cell.type_after = static_cast<uint8_t>(t.terrain_type(cell.cx, cell.cz));
            cell.water_after = t.has_water(cell.cx, cell.cz) ? 1.0f : Terrain::WATER_OFF;
        }
    }
    auto cmd = std::make_unique<WorldEditCommand>(this->world_, label, std::move(this->before_),
                                                  std::move(this->after_), std::move(this->cells_));
    this->before_.clear();
    this->after_.clear();
    this->cells_.clear();
    return cmd;
}

int EditRecorder::find_cell(int cx, int cz) const {
    for (size_t i = 0; i < this->cells_.size(); ++i) {
        if (this->cells_[i].cx == cx && this->cells_[i].cz == cz) return static_cast<int>(i);
    }
    return -1;
}

}  // namespace robcraft::editor
