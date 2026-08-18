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

#include "robcraft/editor/editor_app.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cmath>
#include <unordered_map>
#include <vector>

#include "robcraft/engine/core/data_path.hpp"
#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/renderer/animation_player.hpp"
#include "robcraft/renderer/camera_controls.hpp"
#include "robcraft/renderer/model.hpp"
#include "robcraft/renderer/pick.hpp"
#include "robcraft/renderer/window_icon.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::robots::differential_drive;

void EditorApp::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* self = static_cast<EditorApp*>(glfwGetWindowUserPointer(window));
    if (!self) return;
    if (self->prev_scroll_cb_) self->prev_scroll_cb_(window, xoffset, yoffset);
    self->scroll_offset_ += yoffset;
}

void EditorApp::close_callback(GLFWwindow* window) {
    auto* self = static_cast<EditorApp*>(glfwGetWindowUserPointer(window));
    if (!self) return;
    self->exit_requested_ = true;
    glfwSetWindowShouldClose(window, GLFW_FALSE);
}

EditorApp::EditorApp()
    : undo_recorder_(world_),
      ui_(*this),
      panels_(*this),
      inspector_(*this),
      view_settings_(*this),
      dialogs_(*this),
      viewport_(*this),
      placement_(*this),
      tools_(*this),
      gizmo_tool_(*this) {
    this->brush_radius_by_tool_.fill(2);
}

int& EditorApp::brush_radius() {
    return this->brush_radius_by_tool_[static_cast<size_t>(this->current_tool_)];
}

const int& EditorApp::brush_radius() const {
    return this->brush_radius_by_tool_[static_cast<size_t>(this->current_tool_)];
}
EditorApp::~EditorApp() {
    this->shutdown();
}

bool EditorApp::init() {
    auto log = get_logger();
    log->info("Starting RobCraft Editor");
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    this->window_ = glfwCreateWindow(1400, 900, "RobCraft Editor", nullptr, nullptr);
    if (!this->window_) {
        glfwTerminate();
        return false;
    }
    glfwMaximizeWindow(this->window_);
    robcraft::renderer::set_window_icon(
        this->window_, robcraft::engine::core::resolve_data_path("assets/logo.png"));
    glfwMakeContextCurrent(this->window_);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) return false;
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CLIP_DISTANCE0);
    if (!this->shader_.compile(Shader::default_vertex(), Shader::default_fragment())) return false;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(this->window_, true);
    glfwSetWindowUserPointer(this->window_, this);
    this->prev_scroll_cb_ = glfwSetScrollCallback(this->window_, EditorApp::scroll_callback);
    glfwSetWindowCloseCallback(this->window_, EditorApp::close_callback);
    ImGui_ImplOpenGL3_Init("#version 330");

    this->meshes_ = robcraft::renderer::make_primitive_meshes();

    {
        std::vector<Vertex> rv;
        std::vector<GLuint> ri;
        const int seg = 48;
        for (int i = 0; i < seg; ++i) {
            float a0 =
                static_cast<float>(i) * static_cast<float>(robcraft::engine::math::kTwoPi) / seg;
            float a1 = static_cast<float>(i + 1) *
                       static_cast<float>(robcraft::engine::math::kTwoPi) / seg;
            rv.push_back({std::cos(a0), 0.0f, std::sin(a0), 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f,
                          0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
            rv.push_back({std::cos(a1), 0.0f, std::sin(a1), 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f,
                          0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
            ri.push_back(static_cast<GLuint>(2 * i));
            ri.push_back(static_cast<GLuint>(2 * i + 1));
        }
        this->brush_ring_mesh_.upload(rv, ri, GL_LINES);
    }
    {
        std::vector<Vertex> dv;
        std::vector<GLuint> di;
        const int seg = 48;
        dv.push_back(
            {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        for (int i = 0; i <= seg; ++i) {
            float a =
                static_cast<float>(i) * static_cast<float>(robcraft::engine::math::kTwoPi) / seg;
            dv.push_back({std::cos(a), 0.0f, std::sin(a), 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f,
                          0.0f, 1.0f, 0.0f, 0.0f});
        }
        for (int i = 0; i < seg; ++i) {
            di.push_back(0);
            di.push_back(static_cast<GLuint>(i + 1));
            di.push_back(static_cast<GLuint>(i + 2));
        }
        this->brush_disc_mesh_.upload(dv, di);
    }
    {
        // Unit wireframe cube spanning [-1, 1] for selection highlighting.
        const float c[8][3] = {{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1},
                               {-1, 1, -1},  {1, 1, -1},  {1, 1, 1},  {-1, 1, 1}};
        const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                  {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        std::vector<Vertex> sv;
        std::vector<GLuint> si;
        for (auto& e : edges) {
            float ax = c[e[0]][0], ay = c[e[0]][1], az = c[e[0]][2];
            float bx = c[e[1]][0], by = c[e[1]][1], bz = c[e[1]][2];
            sv.push_back(
                {ax, ay, az, 0.0f, 1.0f, 0.0f, 0.4f, 0.9f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
            sv.push_back(
                {bx, by, bz, 0.0f, 1.0f, 0.0f, 0.4f, 0.9f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
            si.push_back(static_cast<GLuint>(sv.size() - 2));
            si.push_back(static_cast<GLuint>(sv.size() - 1));
        }
        this->selection_box_mesh_.upload(sv, si, GL_LINES);
    }

    this->gizmo_.build();

    this->editor_camera_.set_perspective(60.0f, 1400.0f / 900.0f, 0.1f, 1000.0f);
    this->editor_camera_.set_position(Vec3(50.0, 40.0, 50.0));
    this->editor_camera_.look_at(Vec3(0.0, 0.0, 0.0));

    this->texture_pack_.load(this->texture_size_);

    this->tools_.create_world(32, 32, 1.0);
    this->last_time_ = glfwGetTime();
    return true;
}

void EditorApp::shutdown() {
    if (this->shutdown_done_) return;
    this->shutdown_done_ = true;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    this->shader_.destroy();
    this->meshes_.destroy();
    this->terrain_mesh_.destroy();
    this->texture_pack_.destroy();
    this->grid_mesh_.destroy();
    this->water_mesh_.destroy();
    this->brush_ring_mesh_.destroy();
    this->brush_disc_mesh_.destroy();
    this->selection_box_mesh_.destroy();
    this->gizmo_.destroy();
    this->viewport_fbo_.destroy();
    this->reflection_fbo_.destroy();
    this->shadow_fbo_.destroy();
    if (this->window_) {
        glfwDestroyWindow(this->window_);
        this->window_ = nullptr;
        glfwTerminate();
    }
}

void EditorApp::run() {
    while (this->window_ && !glfwWindowShouldClose(this->window_)) {
        glfwPollEvents();
        double now = glfwGetTime();
        double dt = now - this->last_time_;
        this->last_time_ = now;
        if (dt > 0.1) dt = 0.016;

        {
            // Right-click on the viewport re-pivots the orbit target.
            double mx, my;
            glfwGetCursorPos(this->window_, &mx, &my);
            static bool right_prev = false;
            bool right_now =
                glfwGetMouseButton(this->window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            if (this->viewport_hovered_ && right_now && !right_prev) {
                if (auto pivot = robcraft::renderer::pick_world_point(
                        this->editor_camera_, this->world_, mx, my, this->viewport_x_,
                        this->viewport_y_, this->viewport_w_, this->viewport_h_)) {
                    this->editor_camera_.set_orbit_target(*pivot);
                    this->editor_camera_.look_at(*pivot);
                }
            }
            right_prev = right_now;

            // Gate keyboard camera movement only on text input: ImGui holds
            // WantCaptureKeyboard while a mouse button is down on empty window
            // space (window-move capture), which would block WASD while
            // painting with a brush tool. Shortcut modifiers are handled in
            // handle_camera_input.
            bool keyboard = !ImGui::GetIO().WantTextInput;
            bool mouse = this->viewport_hovered_;
            robcraft::renderer::handle_camera_input(this->window_, this->editor_camera_,
                                                    this->scroll_offset_, 60.0f * (float)dt, 0.01f,
                                                    keyboard, mouse, mouse);
            // Preserve the original: scroll is consumed even when the cursor is off the viewport.
            if (!this->viewport_hovered_) this->scroll_offset_ = 0.0;
        }

        static bool gp = false;
        if (glfwGetKey(this->window_, GLFW_KEY_G) == GLFW_PRESS && !gp) {
            gp = true;
            this->show_grid_ = !this->show_grid_;
        }
        if (glfwGetKey(this->window_, GLFW_KEY_G) == GLFW_RELEASE) gp = false;

        if (glfwGetKey(this->window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            this->drag_active_ = false;
            this->gizmo_tool_.end_gizmo_drag(true);
        }

        static bool dp = false;
        if (glfwGetKey(this->window_, GLFW_KEY_DELETE) == GLFW_PRESS && !dp) {
            dp = true;
            if (!ImGui::GetIO().WantTextInput) this->placement_.delete_selection();
        }
        if (glfwGetKey(this->window_, GLFW_KEY_DELETE) == GLFW_RELEASE) dp = false;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        if (this->exit_requested_) {
            this->exit_requested_ = false;
            this->dialogs_.request_pending_action(PendingAction::Exit);
        }
        this->handle_shortcuts();
        this->ui_.render_ui();
        ImGui::Render();

        int dw, dh;
        glfwGetFramebufferSize(this->window_, &dw, &dh);
        glViewport(0, 0, dw, dh);
        glClearColor(0.1f, 0.15f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(this->window_);
    }
}

void EditorApp::handle_shortcuts() {
    if (ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)) return;
    if (ImGui::GetIO().WantTextInput) return;
    bool ctrl = ImGui::GetIO().KeyCtrl;
    if (ctrl) {
        if (ImGui::IsKeyPressed(ImGuiKey_N, false))
            this->dialogs_.request_pending_action(PendingAction::NewWorld);
        if (ImGui::IsKeyPressed(ImGuiKey_O, false))
            this->dialogs_.request_pending_action(PendingAction::OpenWorld);
        if (ImGui::IsKeyPressed(ImGuiKey_S, false)) {
            if (ImGui::GetIO().KeyShift || this->current_file_.empty())
                this->dialogs_.request_save_world();
            else
                this->tools_.save_world(this->current_file_);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
            if (ImGui::GetIO().KeyShift)
                this->redo();
            else
                this->undo();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Y, false)) this->redo();
        return;
    }
    EditorTool prev_tool = this->current_tool_;
    if (ImGui::IsKeyPressed(ImGuiKey_1, false)) {
        // Pressing 1 again while in Select cycles the gizmo mode (Off ->
        // Move -> Rotate). Off leaves the gizmo hidden for select-only use.
        if (this->current_tool_ == EditorTool::Select) {
            if (this->gizmo_drag_active_) this->gizmo_tool_.end_gizmo_drag(true);
            this->gizmo_tool_.cycle_gizmo_mode();
        } else {
            this->current_tool_ = EditorTool::Select;
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_2, false)) this->current_tool_ = EditorTool::Place;
    if (ImGui::IsKeyPressed(ImGuiKey_3, false)) this->current_tool_ = EditorTool::CliffTerrain;
    if (ImGui::IsKeyPressed(ImGuiKey_4, false)) this->current_tool_ = EditorTool::RaiseTerrain;
    if (ImGui::IsKeyPressed(ImGuiKey_5, false)) this->current_tool_ = EditorTool::LowerTerrain;
    if (ImGui::IsKeyPressed(ImGuiKey_6, false)) this->current_tool_ = EditorTool::FlattenTerrain;
    if (ImGui::IsKeyPressed(ImGuiKey_7, false)) this->current_tool_ = EditorTool::WaterTerrain;
    if (ImGui::IsKeyPressed(ImGuiKey_8, false)) this->current_tool_ = EditorTool::PaintTerrain;
    if (this->current_tool_ != prev_tool) this->drag_active_ = false;
}

void EditorApp::undo() {
    if (this->drag_active_ || this->gizmo_drag_active_ || this->brush_stroke_active_ ||
        ImGui::IsAnyItemActive())
        return;
    if (!this->undo_stack_.can_undo()) return;
    this->undo_stack_.undo();
    this->after_undo_redo();
}

void EditorApp::redo() {
    if (this->drag_active_ || this->gizmo_drag_active_ || this->brush_stroke_active_ ||
        ImGui::IsAnyItemActive())
        return;
    if (!this->undo_stack_.can_redo()) return;
    this->undo_stack_.redo();
    this->after_undo_redo();
}

void EditorApp::after_undo_redo() {
    std::vector<Entity> keep;
    for (Entity e : this->selection_) {
        if (this->world_.valid(e)) keep.push_back(e);
    }
    this->selection_ = std::move(keep);
    if (!this->world_.valid(this->hovered_entity_)) this->hovered_entity_ = INVALID_ENTITY;
    this->gizmo_drag_active_ = false;
    this->gizmo_handle_ = GizmoHandle::None;
    this->gizmo_start_transforms_.clear();
    this->brush_stroke_active_ = false;
    this->undo_recorder_.begin();
    // The world changed outside an inspector edit session; drop the stale
    // baseline so the next inspector edit captures a fresh before-state.
    this->inspector_before_.reset();
    if (this->world_.has_terrain()) {
        this->viewport_.rebuild_terrain_mesh();
        this->viewport_.rebuild_grid_mesh();
    }
    this->tools_.mark_modified();
}

void EditorApp::commit_undo(const char* label) {
    auto cmd = this->undo_recorder_.finish(label);
    if (cmd) this->undo_stack_.execute(std::move(cmd));
}

void EditorApp::commit_inspector_edit(Entity e) {
    if (!this->inspector_before_) return;
    auto after = EntitySnapshot::capture(this->world_, e);
    if (!after) {
        this->inspector_before_.reset();
        return;
    }
    std::vector<std::unique_ptr<EntitySnapshot>> before_list;
    before_list.push_back(std::move(this->inspector_before_));
    std::vector<std::unique_ptr<EntitySnapshot>> after_list;
    after_list.push_back(std::move(after));
    this->undo_stack_.execute(
        std::make_unique<WorldEditCommand>(this->world_, "Edit property", std::move(before_list),
                                           std::move(after_list), std::vector<TerrainCellDelta>{}));
    this->inspector_before_.reset();
}

SceneDrawContext EditorApp::make_scene_ctx() {
    return SceneDrawContext{this->shader_, this->model_cache_, this->texture_pack_, this->meshes_};
}

const std::vector<Mat4>* EditorApp::editor_skin_matrices(
    Entity /*e*/, const std::shared_ptr<Model>& model) const {
    if (!model || !model->skinned()) return nullptr;
    static std::unordered_map<const Model*, AnimationPlayer> players;
    static std::unordered_map<const Model*, double> last_times;
    double now = glfwGetTime();
    double last = last_times[model.get()];
    double dt = (last > 0.0) ? (now - last) : 0.016;
    last_times[model.get()] = now;
    auto& player = players[model.get()];
    if (!player.has_clip()) {
        player.set_model(model);
        player.play("Idle");
    }
    player.update(dt);
    return &player.joint_matrices();
}

}  // namespace robcraft::editor

int main(int argc, char* argv[]) {
    auto logger = robcraft::engine::core::init_logger();
    logger->info("RobCraft Editor v1.1.0");
    robcraft::editor::EditorApp app;
    if (!app.init()) {
        logger->error("Init failed");
        return 1;
    }
    app.run();
    app.shutdown();
    return 0;
}
