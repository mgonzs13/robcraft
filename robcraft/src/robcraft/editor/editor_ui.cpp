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

#include "robcraft/editor/editor_ui.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <string>

#include "robcraft/editor/editor_app.hpp"

namespace robcraft::editor {

using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::robots::differential_drive;

EditorUi::EditorUi(EditorApp& app) : app_(app) {}

void EditorUi::render_ui() {
    this->render_menu_bar();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float top = vp->Pos.y + 20.0f;
    const float bar_h = 30.0f;
    const float opts_h = 30.0f;
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, top));
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x, bar_h));
    this->render_toolbar();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, top + bar_h));
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x, opts_h));
    this->app_.panels_.render_tool_options();
    float lw = vp->Size.x * 0.18f, rw = vp->Size.x * 0.25f;
    float cw = vp->Size.x - lw - rw, bh = 25;
    const float my = top + bar_h + opts_h;
    float mh = vp->Size.y - my - bh;
    const float ph = (this->app_.current_tool_ == EditorTool::Place) ? mh * 0.35f : 0.0f;
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, my));
    ImGui::SetNextWindowSize(ImVec2(lw, ph));
    if (this->app_.current_tool_ == EditorTool::Place) this->app_.panels_.render_palette();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, my + ph));
    ImGui::SetNextWindowSize(ImVec2(lw, mh - ph));
    this->app_.panels_.render_hierarchy();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + lw, my));
    ImGui::SetNextWindowSize(ImVec2(cw, mh));
    this->app_.viewport_.render_viewport();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + lw + cw, my));
    ImGui::SetNextWindowSize(ImVec2(rw, mh));
    this->app_.inspector_.render_inspector();
    this->app_.view_settings_.render_water_panel();
    this->app_.view_settings_.render_lighting_panel();
    this->app_.view_settings_.render_sky_panel();
    this->app_.view_settings_.render_physics_panel();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, vp->Pos.y + vp->Size.y - bh));
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x, bh));
    this->render_status_bar();
    this->app_.dialogs_.render_dialogs();
}

void EditorUi::render_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New", "Ctrl+N"))
                this->app_.dialogs_.request_pending_action(PendingAction::NewWorld);
            if (ImGui::MenuItem("Open...", "Ctrl+O"))
                this->app_.dialogs_.request_pending_action(PendingAction::OpenWorld);
            if (ImGui::MenuItem("Save", "Ctrl+S")) {
                if (this->app_.current_file_.empty())
                    this->app_.dialogs_.request_save_world();
                else
                    this->app_.tools_.save_world(this->app_.current_file_);
            }
            if (ImGui::MenuItem("Save As...", "Ctrl+Shift+S"))
                this->app_.dialogs_.request_save_world();
            ImGui::Separator();
            if (ImGui::MenuItem("Options...")) this->app_.show_options_ = true;
            ImGui::Separator();
            if (ImGui::MenuItem("Exit"))
                this->app_.dialogs_.request_pending_action(PendingAction::Exit);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit")) {
            const char* ul = this->app_.undo_stack_.undo_label();
            std::string undo_label = ul ? "Undo: " + std::string(ul) : "Undo";
            if (ImGui::MenuItem(undo_label.c_str(), "Ctrl+Z", false,
                                this->app_.undo_stack_.can_undo()))
                this->app_.undo();
            const char* rl = this->app_.undo_stack_.redo_label();
            std::string redo_label = rl ? "Redo: " + std::string(rl) : "Redo";
            if (ImGui::MenuItem(redo_label.c_str(), "Ctrl+Shift+Z", false,
                                this->app_.undo_stack_.can_redo()))
                this->app_.redo();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Grid (G)", nullptr, &this->app_.show_grid_);
            if (ImGui::MenuItem("Water Settings")) this->app_.show_water_panel_ = true;
            if (ImGui::MenuItem("Lighting")) this->app_.show_lighting_panel_ = true;
            if (ImGui::MenuItem("Sky")) this->app_.show_sky_panel_ = true;
            if (ImGui::MenuItem("Physics")) this->app_.show_physics_panel_ = true;
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void EditorUi::render_toolbar() {
    ImGui::Begin("Toolbar", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(12.0f, 8.0f));
    auto tool = [this](const char* name, EditorTool t, const char* tip) {
        std::string label = this->app_.current_tool_ == t ? "[" + std::string(name) + "]" : name;
        if (ImGui::Button(label.c_str())) {
            this->app_.current_tool_ = t;
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
    };
    tool("Select", EditorTool::Select, "Select entities (1)");
    ImGui::SameLine();
    tool("Place", EditorTool::Place, "Buildings, props and robots from the palette (2)");
    ImGui::SameLine();
    tool("Cliff", EditorTool::CliffTerrain, "Set cliff height to the target level (3)");
    ImGui::SameLine();
    tool("Raise", EditorTool::RaiseTerrain, "Smooth continuous raise (4)");
    ImGui::SameLine();
    tool("Lower", EditorTool::LowerTerrain, "Smooth continuous lower (5)");
    ImGui::SameLine();
    tool("Flat", EditorTool::FlattenTerrain, "Flatten toward the height under the cursor (6)");
    ImGui::SameLine();
    tool("Water", EditorTool::WaterTerrain, "Paint water at the terrain height (7)");
    ImGui::SameLine();
    tool("Paint", EditorTool::PaintTerrain, "Paint terrain type (8)");
    ImGui::PopStyleVar();
    ImGui::End();
}

void EditorUi::render_status_bar() {
    ImGui::Begin("Status", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus);
    size_t cnt = 0, m = this->app_.world_.entities().max_allocated();
    for (size_t i = 1; i <= m; ++i)
        if (this->app_.world_.valid((Entity)i)) cnt++;
    const char* tn = "Select";
    switch (this->app_.current_tool_) {
        case EditorTool::Select:
            tn = "Select";
            break;
        case EditorTool::Place:
            tn = "Place";
            break;
        case EditorTool::RaiseTerrain:
            tn = "Raise";
            break;
        case EditorTool::LowerTerrain:
            tn = "Lower";
            break;
        case EditorTool::FlattenTerrain:
            tn = "Flat";
            break;
        case EditorTool::CliffTerrain:
            tn = "Cliff";
            break;
        case EditorTool::WaterTerrain:
            tn = "Water";
            break;
        case EditorTool::PaintTerrain:
            tn = "Paint";
            break;
    }
    ImGui::Text("%zu entities | %s | %s", cnt, tn,
                this->app_.current_file_.empty() ? "(new)" : this->app_.current_file_.c_str());
    ImGui::End();
}

}  // namespace robcraft::editor
