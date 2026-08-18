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

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <array>
#include <string>
#include <vector>

#include "robcraft/editor/command/command.hpp"
#include "robcraft/editor/command/entity_snapshot.hpp"
#include "robcraft/editor/command/world_edit_command.hpp"
#include "robcraft/editor/editor_dialogs.hpp"
#include "robcraft/editor/editor_gizmo.hpp"
#include "robcraft/editor/editor_inspector.hpp"
#include "robcraft/editor/editor_panels.hpp"
#include "robcraft/editor/editor_place.hpp"
#include "robcraft/editor/editor_tools.hpp"
#include "robcraft/editor/editor_ui.hpp"
#include "robcraft/editor/editor_undo.hpp"
#include "robcraft/editor/editor_view_settings.hpp"
#include "robcraft/editor/editor_viewport.hpp"
#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/core/random.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/math/cell_range.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/camera.hpp"
#include "robcraft/renderer/fbo.hpp"
#include "robcraft/renderer/mesh.hpp"
#include "robcraft/renderer/model.hpp"
#include "robcraft/renderer/model_paths.hpp"
#include "robcraft/renderer/primitive_meshes.hpp"
#include "robcraft/renderer/scene_render.hpp"
#include "robcraft/renderer/shader.hpp"
#include "robcraft/renderer/texture.hpp"
#include "robcraft/renderer/texture_pack.hpp"

namespace robcraft::editor {

using namespace robcraft::editor::command;
using namespace robcraft::engine::collision;
using namespace robcraft::engine::core;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;

/** @brief Currently active editor tool. */
enum class EditorTool {
    Select,
    Place,
    RaiseTerrain,
    LowerTerrain,
    FlattenTerrain,
    CliffTerrain,
    WaterTerrain,
    PaintTerrain,
    /** @brief Sentinel value equal to the number of tools. */
    Count,
};

/** @brief Placeable types in the unified Place tool palette. */
enum class PlaceableType {
    // Buildings
    /** @brief Grid-aligned wall segment or run (procedural cube). */
    Wall = 0,
    /** @brief Walkable floor rectangle (procedural flat plate). */
    Floor = 1,
    // Furniture
    /** @brief Bed. */
    Bed = 2,
    /** @brief Chair. */
    Chair = 3,
    /** @brief Large couch. */
    Couch = 4,
    /** @brief Floor lamp. */
    LightFloor = 5,
    /** @brief Shelf. */
    Shelf = 6,
    /** @brief Table. */
    Table = 7,
    // Nature
    /** @brief Common tree 1. */
    Tree = 8,
    /** @brief Common tree 2. */
    Tree2 = 9,
    /** @brief Common tree 3. */
    Tree3 = 10,
    /** @brief Common tree 4. */
    Tree4 = 11,
    /** @brief Common tree 5. */
    Tree5 = 12,
    /** @brief Pine tree 1. */
    Pine1 = 13,
    /** @brief Pine tree 2. */
    Pine2 = 14,
    /** @brief Pine tree 3. */
    Pine3 = 15,
    /** @brief Pine tree 4. */
    Pine4 = 16,
    /** @brief Pine tree 5. */
    Pine5 = 17,
    /** @brief Twisted tree 1. */
    Twisted1 = 18,
    /** @brief Twisted tree 2. */
    Twisted2 = 19,
    /** @brief Twisted tree 3. */
    Twisted3 = 20,
    /** @brief Twisted tree 4. */
    Twisted4 = 21,
    /** @brief Twisted tree 5. */
    Twisted5 = 22,
    /** @brief Dead tree 1. */
    Dead1 = 23,
    /** @brief Dead tree 2. */
    Dead2 = 24,
    /** @brief Dead tree 3. */
    Dead3 = 25,
    /** @brief Dead tree 4. */
    Dead4 = 26,
    /** @brief Dead tree 5. */
    Dead5 = 27,
    /** @brief Bush. */
    Bush = 28,
    /** @brief Bush with flowers. */
    Bush2 = 29,
    /** @brief Rock 1 (textured). */
    Rock1 = 30,
    /** @brief Rock 2 (textured). */
    Rock2 = 31,
    /** @brief Rock 3 (textured). */
    Rock3 = 32,
    /** @brief Rock 4 (textured). */
    Rock4 = 33,
    /** @brief Rock Large 1 (textured). */
    RockLarge1 = 34,
    /** @brief Rock Large 2 (textured). */
    RockLarge2 = 35,
    /** @brief Rock Large 3 (textured). */
    RockLarge3 = 36,
    // Space
    /** @brief Moon rock (textured). */
    MoonRock = 37,
    /** @brief Moon rock 2 (textured). */
    MoonRock2 = 38,
    /** @brief Moon rock 3 (textured). */
    MoonRock3 = 39,
    /** @brief Large moon rock (textured). */
    MoonRockLarge = 40,
    /** @brief Space base (multi-tile, realistic size). */
    SpaceBase = 41,
    /** @brief Geodesic dome. */
    GeodesicDome = 42,
    /** @brief Ground solar panel. */
    SolarPanel = 43,
    // Robots
    /** @brief Robot: George mech. */
    RobotGeorge = 44,
    /** @brief Robot: Leela mech. */
    RobotLeela = 45,
    /** @brief Robot: Mike mech. */
    RobotMike = 46,
    /** @brief Robot: Stan mech. */
    RobotStan = 47,
    /** @brief Point light (placeable illumination). */
    PointLight = 48,
    // Animals
    /** @brief Cow farm animal (skinned, Idle animation). */
    Cow = 49,
    /** @brief Horse farm animal (skinned, Idle animation). */
    Horse = 50,
    /** @brief Llama farm animal (skinned, Idle animation). */
    Llama = 51,
    /** @brief Pig farm animal (skinned, Idle animation). */
    Pig = 52,
    /** @brief Pug farm animal (skinned, Idle animation). */
    Pug = 53,
    /** @brief Sheep farm animal (skinned, Idle animation). */
    Sheep = 54,
    /** @brief Zebra farm animal (skinned, Idle animation). */
    Zebra = 55,
    /** @brief Sentinel value. */
    Count = 56
};

/** @brief Mode of the path fallback modal. */
enum class FileDialogMode {
    /** @brief Picking a .world file to open. */
    Open,
    /** @brief Picking a destination path for saving. */
    SaveAs,
};

/** @brief Action deferred while the unsaved-changes dialog is shown. */
enum class PendingAction {
    /** @brief No action pending. */
    None,
    /** @brief Open the New World dialog after confirmation. */
    NewWorld,
    /** @brief Open a world via the file dialog. */
    OpenWorld,
    /** @brief Close the application after confirmation. */
    Exit,
};

/** @brief Main editor application: window, viewport, and world editing tools. */
class EditorApp {
public:
    /** @brief Constructs the editor application. */
    EditorApp();
    /** @brief Destroys the editor and releases resources. */
    ~EditorApp();

    /** @brief Initializes GLFW, OpenGL, ImGui, meshes, and the default world. */
    bool init();
    /** @brief Runs the main loop until the window is closed. */
    void run();
    /** @brief Releases all editor resources. */
    void shutdown();

    /** @brief GLFW scroll callback that forwards to ImGui and accumulates camera zoom.
     *  @param window The GLFW window that received the scroll event.
     *  @param xoffset Horizontal scroll offset.
     *  @param yoffset Vertical scroll offset. */
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    /** @brief GLFW window close callback that routes through the unsaved-changes guard. */
    static void close_callback(GLFWwindow* window);

private:
    friend class EditorUi;
    friend class EditorPanels;
    friend class EditorInspector;
    friend class EditorViewSettings;
    friend class EditorDialogs;
    friend class EditorViewport;
    friend class EditorPlacement;
    friend class EditorTools;
    friend class EditorGizmo;

    /** @brief Returns the active tool's brush radius.
     *  @return Mutable reference to the current tool's brush radius. */
    int& brush_radius();
    /** @brief Returns the active tool's brush radius.
     *  @return Const reference to the current tool's brush radius. */
    const int& brush_radius() const;

    /** @brief Handles keyboard shortcuts (tools, file ops) via ImGui IO. */
    void handle_shortcuts();
    /** @brief Undoes the most recent command, if any. */
    void undo();
    /** @brief Re-applies the most recently undone command, if any. */
    void redo();
    /** @brief Fixes up selection, hover, and gizmo state after undo/redo. */
    void after_undo_redo();
    /** @brief Captures the current world state and pushes it as a command.
     *  @param label Short label for the command. */
    void commit_undo(const char* label);
    /** @brief Commits an inspector edit session as an undo command.
     *  @param e The edited entity. */
    void commit_inspector_edit(Entity e);
    /** @brief Returns the shared draw context for this editor.
     *  @return Context referencing the editor's shader, models, textures, and meshes. */
    SceneDrawContext make_scene_ctx();
    /** @brief Editor skin lookup: always-playing per-model animation players.
     *  @param e The entity being drawn (unused).
     *  @param model The resolved model.
     *  @return The model's joint matrices, or null when not skinned. */
    const std::vector<Mat4>* editor_skin_matrices(Entity e,
                                                  const std::shared_ptr<Model>& model) const;

    GLFWwindow* window_ = nullptr;

    /** @brief The world being edited. */
    World world_;
    /** @brief Undo/redo command history. */
    UndoStack undo_stack_;
    /** @brief Captures the current gesture's before/after state. */
    EditRecorder undo_recorder_;
    /** @brief Before-snapshot for the current inspector edit session, if any. */
    std::unique_ptr<EntitySnapshot> inspector_before_;
    /** @brief Deterministic random source for doodad placement. */
    Random rng_{42};
    /** @brief Currently selected editor tool. */
    EditorTool current_tool_ = EditorTool::Select;
    /** @brief Currently selected entities; the last element is the primary. */
    std::vector<Entity> selection_;
    /** @brief Entity under the cursor for hover highlighting. */
    Entity hovered_entity_ = INVALID_ENTITY;
    /** @brief Current placeable for the Place tool. */
    PlaceableType placeable_ = PlaceableType::Wall;
    /** @brief Whether a wall/floor drag is in progress. */
    bool drag_active_ = false;
    /** @brief Whether a terrain brush stroke is in progress. */
    bool brush_stroke_active_ = false;
    /** @brief Anchor cell of a drag in progress. */
    int drag_anchor_x_ = 0;
    /** @brief Anchor cell of a drag in progress. */
    int drag_anchor_z_ = 0;
    /** @brief Current cursor cell during a drag. */
    int drag_cur_x_ = 0;
    /** @brief Current cursor cell during a drag. */
    int drag_cur_z_ = 0;

    /** @brief Path of the currently open world file. */
    std::string current_file_;

    /** @brief Shader used to render the scene. */
    Shader shader_;
    /** @brief Free-fly camera for the 3D viewport. */
    Camera editor_camera_;
    /** @brief Terrain mesh rebuilt from height data. */
    Mesh terrain_mesh_;
    /** @brief Shared procedural fallback meshes. */
    PrimitiveMeshes meshes_;
    /** @brief Process-wide model cache (OBJ load once). */
    ModelCache model_cache_;
    /** @brief Shared terrain/building textures. */
    TexturePack texture_pack_;
    /** @brief Shared terrain/building texture size in pixels (256/512/1024). */
    int texture_size_ = 256;
    /** @brief Grid overlay line mesh (cell borders only, no diagonals). */
    Mesh grid_mesh_;
    /** @brief Per-cell water surface mesh. */
    Mesh water_mesh_;
    /** @brief Unit circle ring for brush previews. */
    Mesh brush_ring_mesh_;
    /** @brief Unit disc for brush previews. */
    Mesh brush_disc_mesh_;
    /** @brief Unit wireframe cube for selection highlights. */
    Mesh selection_box_mesh_;
    /** @brief Transform gizmo (move arrows / yaw ring). */
    Gizmo gizmo_;
    /** @brief Active gizmo mode from the Select-tool toolbar toggle. Defaults to
     *  Off so a fresh editor session is select-only. */
    GizmoMode gizmo_mode_ = GizmoMode::Off;
    /** @brief Whether a gizmo handle is being dragged. */
    bool gizmo_drag_active_ = false;
    /** @brief The handle being dragged. */
    GizmoHandle gizmo_handle_ = GizmoHandle::None;
    /** @brief World point on the drag plane captured at press. */
    Vec3 gizmo_grab_pt_;
    /** @brief Primary selection position at press. */
    Vec3 gizmo_center_;
    /** @brief Yaw of the cursor from the previous drag frame (rotate mode). */
    double gizmo_last_angle_ = 0.0;
    /** @brief Accumulated rotation since grab, unwrapped across the atan2 seam. */
    double gizmo_drag_angle_ = 0.0;
    /** @brief Start transforms of every selected entity at press. */
    std::vector<std::pair<Entity, Transform3D>> gizmo_start_transforms_;
    /** @brief FBO holding the rendered viewport. */
    FBO viewport_fbo_;
    /** @brief FBO holding the planar water reflection pass. */
    FBO reflection_fbo_;
    /** @brief FBO holding the sun's shadow map. */
    FBO shadow_fbo_;
    /** @brief Whether planar water reflections are rendered. */
    bool water_reflection_ = false;
    /** @brief Reflection blend strength (0..1). */
    float water_reflection_strength_ = 0.85f;
    /** @brief Whether the terrain mesh has been built. */
    bool has_terrain_mesh_ = false;
    /** @brief Whether the grid overlay is enabled. */
    bool show_grid_ = true;
    /** @brief Whether the mouse is hovering the 3D viewport this frame. */
    bool viewport_hovered_ = false;

    /** @brief Viewport rectangle in window coordinates, updated each frame. */
    int viewport_x_ = 0;
    /** @brief Viewport rectangle in window coordinates, updated each frame. */
    int viewport_y_ = 0;
    /** @brief Viewport rectangle in window coordinates, updated each frame. */
    int viewport_w_ = 0;
    /** @brief Viewport rectangle in window coordinates, updated each frame. */
    int viewport_h_ = 0;

    /** @brief Time of the previous frame for delta-time computation. */
    double last_time_ = 0.0;
    /** @brief Accumulated scroll wheel offset, consumed each frame as camera zoom. */
    double scroll_offset_ = 0.0;
    /** @brief Scroll callback ImGui installed, chained from our callback. */
    GLFWscrollfun prev_scroll_cb_ = nullptr;
    /** @brief Whether the OS requested window close and it is pending confirmation. */
    bool exit_requested_ = false;
    /** @brief Whether shutdown has already run (idempotency guard). */
    bool shutdown_done_ = false;
    /** @brief Terrain brush radius in cells, one per tool (default 2). */
    std::array<int, static_cast<size_t>(EditorTool::Count)> brush_radius_by_tool_;
    /** @brief Terrain paint type for the paint tool. */
    TerrainType paint_type_ = TerrainType::Grass;
    /** @brief Whether the Water tool erases water instead of painting. */
    bool water_clear_ = false;
    /** @brief Water animation speed multiplier (view setting, not world data). */
    float water_speed_ = 0.30f;
    /** @brief Water wave amplitude / gradient contrast (view setting). */
    float water_wave_amp_ = 0.50f;
    /** @brief Water specular highlight strength (view setting). */
    float water_specular_ = 0.00f;
    /** @brief Water opacity / alpha (view setting). */
    float water_opacity_ = 0.72f;
    /** @brief Shoreline foam intensity (view setting). */
    float water_foam_ = 0.60f;
    /** @brief Shoreline foam band width in meters (view setting). */
    float water_foam_width_ = 1.50f;
    /** @brief Whether the Water settings panel is visible. */
    bool show_water_panel_ = false;
    /** @brief Whether the Lighting settings panel is visible. */
    bool show_lighting_panel_ = false;
    /** @brief Whether the Sky settings panel is visible. */
    bool show_sky_panel_ = false;
    /** @brief Whether the Physics settings panel is visible. */
    bool show_physics_panel_ = false;
    /** @brief Whether the Options modal is visible. */
    bool show_options_ = false;
    /** @brief Pending texture-size combo index in the Options modal (persists
     *  across frames until Apply commits it). */
    int options_texture_index_ = 0;
    /** @brief Whether the world has unsaved changes. */
    bool world_modified_ = false;
    /** @brief Brush strength for the Raise/Lower tools. */
    float brush_strength_ = 0.15f;
    /** @brief Terrain height captured when a Flatten drag begins. */
    float flatten_target_ = 0.0f;
    /** @brief Target cliff level for the Cliff tool (0-16). */
    int cliff_target_level_ = 1;
    /** @brief New World width in cells. */
    int new_world_w_ = 32;
    /** @brief New World height in cells. */
    int new_world_h_ = 32;
    /** @brief New World cell size in meters. */
    float new_world_cell_ = 1.0f;
    /** @brief Mode of the path fallback modal (zenity absent). */
    FileDialogMode fallback_mode_ = FileDialogMode::Open;
    /** @brief Whether the path fallback modal is visible. */
    bool fallback_open_ = false;
    /** @brief Action deferred behind the unsaved-changes dialog. */
    PendingAction pending_action_ = PendingAction::None;
    /** @brief True once the user answered the save prompt for the pending action. */
    bool save_prompt_answered_ = false;

    /** @brief Main UI shell (menu bar, toolbar, status bar). */
    EditorUi ui_;
    /** @brief Tool-options bar, palette, and hierarchy panels. */
    EditorPanels panels_;
    /** @brief Inspector panel for the selected entity. */
    EditorInspector inspector_;
    /** @brief Water, lighting, and sky settings panels. */
    EditorViewSettings view_settings_;
    /** @brief Modal file/world dialogs. */
    EditorDialogs dialogs_;
    /** @brief 3D viewport rendering and mesh rebuilds. */
    EditorViewport viewport_;
    /** @brief Object placement and selection tools. */
    EditorPlacement placement_;
    /** @brief World lifecycle and file operations. */
    EditorTools tools_;
    /** @brief Transform gizmo interaction. */
    EditorGizmo gizmo_tool_;
};

}  // namespace robcraft::editor
