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

#include "robcraft/editor/editor_tools.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/world/world_serializer.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::world;
using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::robots::differential_drive;

EditorTools::EditorTools(EditorApp& app) : app_(app) {}

void EditorTools::create_world(int width, int height, double cell_size) {
    this->app_.world_.clear();
    this->app_.world_.set_terrain(Terrain(width, height, cell_size));
    this->app_.viewport_.rebuild_terrain_mesh();
    this->app_.viewport_.rebuild_grid_mesh();
    this->app_.placement_.clear_selection();
    this->app_.drag_active_ = false;
    this->app_.current_file_.clear();
    this->app_.world_modified_ = false;
    this->app_.undo_stack_.clear();
    this->app_.inspector_before_.reset();
    this->app_.undo_recorder_.begin();
    // Reset the editor's water view settings (not world data) to their defaults.
    this->app_.water_speed_ = 0.30f;
    this->app_.water_wave_amp_ = 0.50f;
    this->app_.water_specular_ = 0.00f;
    this->app_.water_opacity_ = 0.72f;
    this->app_.water_foam_ = 0.60f;
    this->app_.water_foam_width_ = 1.50f;
    this->app_.water_reflection_ = false;
    this->app_.water_reflection_strength_ = 0.85f;
    this->frame_world();
}

void EditorTools::frame_world() {
    if (!this->app_.world_.has_terrain()) return;
    double cs = this->app_.world_.terrain().cell_size();
    double hx = this->app_.world_.terrain().width() * cs * 0.5;
    double hz = this->app_.world_.terrain().height() * cs * 0.5;
    double d = std::max(hx, hz);
    Vec3 center(0.0, 0.0, 0.0);
    this->app_.editor_camera_.set_orbit_target(center);
    this->app_.editor_camera_.set_position(Vec3(d * 1.2, d * 1.0, d * 1.2));
    this->app_.editor_camera_.look_at(center);
}

void EditorTools::mark_modified() {
    this->app_.world_modified_ = true;
}

bool EditorTools::open_world(const std::string& path) {
    World w;
    if (!WorldSerializer::load(w, path)) return false;
    robcraft::renderer::refit_world_colliders(w, this->app_.model_cache_);
    this->app_.world_ = std::move(w);
    this->app_.current_file_ = path;
    this->app_.world_modified_ = false;
    this->app_.undo_stack_.clear();
    this->app_.inspector_before_.reset();
    this->app_.undo_recorder_.begin();
    this->app_.placement_.clear_selection();
    this->app_.drag_active_ = false;
    if (this->app_.world_.has_terrain()) this->app_.viewport_.rebuild_terrain_mesh();
    this->app_.viewport_.rebuild_grid_mesh();
    return true;
}

bool EditorTools::save_world(const std::string& path) {
    if (WorldSerializer::save(this->app_.world_, path)) {
        this->app_.current_file_ = path;
        this->app_.world_modified_ = false;
        return true;
    }
    return false;
}

}  // namespace robcraft::editor
