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

#include "robcraft/engine/core/entity.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::core;

class EditorApp;

/** @brief Renders the 3D viewport and rebuilds its scene meshes. */
class EditorViewport {
public:
    /** @brief Constructs the viewport.
     *  @param app The owning editor application. */
    explicit EditorViewport(EditorApp& app);

    /** @brief Renders the 3D viewport panel. */
    void render_viewport();
    /** @brief Renders the 3D scene into the viewport FBO. */
    void render_3d_to_fbo(int vp_w, int vp_h);
    /** @brief Renders ghosts, brush previews, and selection/hover highlights. */
    void render_previews();
    /** @brief Draws a bright wireframe box around a selected entity.
     *  @param e The entity to highlight. */
    void draw_selection_box(robcraft::engine::core::Entity e) const;
    /** @brief Handles a left-click on the viewport for selection or robot placement. */
    void handle_mouse_pick(int vp_x, int vp_y, int vp_w, int vp_h);
    /** @brief Handles terrain brush tools dragged over the viewport. */
    void handle_terrain_tool();

    /** @brief Rebuilds the grid line mesh from terrain cell borders. */
    void rebuild_grid_mesh();
    /** @brief Rebuilds the water surface mesh from the terrain's per-cell water. */
    void rebuild_water_mesh();
    /** @brief Rebuilds the terrain mesh from the current height data. */
    void rebuild_terrain_mesh();

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
