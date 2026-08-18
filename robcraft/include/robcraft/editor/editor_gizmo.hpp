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

#include "robcraft/editor/gizmo_math.hpp"
#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/mesh.hpp"
#include "robcraft/renderer/shader.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::math;
using namespace robcraft::renderer;

class EditorApp;

/** @brief Gizmo interaction mode. */
enum class GizmoMode {
    /** @brief Gizmo hidden. */
    Off,
    /** @brief Translate arrows (X/Y/Z). */
    Move,
    /** @brief Yaw rotation ring. */
    Rotate
};

/** @brief A draggable gizmo handle. */
enum class GizmoHandle {
    /** @brief Nothing grabbed. */
    None,
    /** @brief X translate arrow. */
    AxisX,
    /** @brief Y translate arrow. */
    AxisY,
    /** @brief Z translate arrow. */
    AxisZ,
    /** @brief Yaw rotation ring. */
    Yaw
};

/** @brief The gizmo: owned meshes, rendering, and handle hit-testing. */
class Gizmo {
public:
    /** @brief Builds the arrow and ring meshes. Requires a GL context. */
    void build();
    /** @brief Destroys the meshes. */
    void destroy();

    /**
     * @brief Draws the gizmo at a world position with a screen-constant size.
     * @param shader The active shader.
     * @param mode Gizmo mode (Off draws nothing).
     * @param center World position of the gizmo.
     * @param world_size World-space gizmo length.
     */
    void render(Shader& shader, GizmoMode mode, const Vec3& center, double world_size) const;

    /**
     * @brief Hit-tests the handles against the cursor and returns the grabbed one.
     * @param cam The camera.
     * @param vp_x Viewport origin x (window coords).
     * @param vp_y Viewport origin y (window coords).
     * @param vp_w Viewport width in pixels.
     * @param vp_h Viewport height in pixels.
     * @param mx Cursor x (window coords).
     * @param my Cursor y (window coords).
     * @param mode Gizmo mode.
     * @param center World position of the gizmo.
     * @param world_size World-space gizmo length.
     * @return The grabbed handle, or GizmoHandle::None.
     */
    GizmoHandle pick(const Camera& cam, int vp_x, int vp_y, int vp_w, int vp_h, double mx,
                     double my, GizmoMode mode, const Vec3& center, double world_size) const;

private:
    /** @brief Red arrow along +X. */
    Mesh arrow_x_;
    /** @brief Green arrow along +Y. */
    Mesh arrow_y_;
    /** @brief Blue arrow along +Z. */
    Mesh arrow_z_;
    /** @brief Green yaw ring in the XZ plane. */
    Mesh ring_;
};

/** @brief Editor gizmo interaction: rendering, handle grabbing, and drag state. */
class EditorGizmo {
public:
    /** @brief Constructs the gizmo interaction.
     *  @param app The owning editor application. */
    explicit EditorGizmo(EditorApp& app);

    /** @brief Draws the transform gizmo on the primary selection. */
    void render_gizmo();
    /** @brief Hit-tests and grabs a gizmo handle on left-click; true if grabbed.
     *  @return True if a handle was grabbed (entity pick should be skipped). */
    bool handle_gizmo_press();
    /** @brief Applies the current gizmo drag to the selection each frame. */
    void update_gizmo_drag();
    /** @brief Ends a gizmo drag.
     *  @param cancel True restores the start transforms, false commits them. */
    void end_gizmo_drag(bool cancel);
    /** @brief Advances the gizmo mode around the Move -> Rotate -> Off cycle. */
    void cycle_gizmo_mode();

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
