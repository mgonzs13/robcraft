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

#include "robcraft/editor/editor_panels.hpp"

#include <imgui.h>

#include <algorithm>
#include <string>
#include <vector>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::ecs;
using namespace robcraft::robots::differential_drive;

EditorPanels::EditorPanels(EditorApp& app) : app_(app) {}

void EditorPanels::render_tool_options() {
    ImGui::Begin("Tool Options", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(12.0f, 8.0f));
    switch (this->app_.current_tool_) {
        case EditorTool::Select:
            ImGui::Text("Gizmo:");
            ImGui::SameLine();
            if (ImGui::Button(this->app_.gizmo_mode_ == GizmoMode::Off ? "[Off]" : "Off"))
                this->app_.gizmo_mode_ = GizmoMode::Off;
            ImGui::SameLine();
            if (ImGui::Button(this->app_.gizmo_mode_ == GizmoMode::Move ? "[Move]" : "Move"))
                this->app_.gizmo_mode_ = GizmoMode::Move;
            ImGui::SameLine();
            if (ImGui::Button(this->app_.gizmo_mode_ == GizmoMode::Rotate ? "[Rotate]" : "Rotate"))
                this->app_.gizmo_mode_ = GizmoMode::Rotate;
            ImGui::SameLine(0.0f, 18.0f);
            ImGui::TextDisabled("Press 1 to cycle gizmo mode");
            break;
        case EditorTool::Place:
            ImGui::TextDisabled("Pick a type from the Palette");
            break;
        case EditorTool::CliffTerrain: {
            ImGui::Text("Brush:");
            ImGui::SameLine();
            ImGui::SliderInt("##br", &this->app_.brush_radius(), 0, 5);
            ImGui::SameLine();
            const char* levels[] = {"0", "1",  "2",  "3",  "4",  "5",  "6",  "7", "8",
                                    "9", "10", "11", "12", "13", "14", "15", "16"};
            ImGui::Combo("Cliff level", &this->app_.cliff_target_level_, levels, 17);
            break;
        }
        case EditorTool::WaterTerrain: {
            ImGui::Text("Brush:");
            ImGui::SameLine();
            ImGui::SliderInt("##br", &this->app_.brush_radius(), 0, 5);
            ImGui::SameLine();
            ImGui::Checkbox("Clear", &this->app_.water_clear_);
            break;
        }
        case EditorTool::RaiseTerrain:
        case EditorTool::LowerTerrain: {
            ImGui::Text("Brush:");
            ImGui::SameLine();
            ImGui::SliderInt("##br", &this->app_.brush_radius(), 0, 5);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(48);
            if (ImGui::InputInt("##br_val", &this->app_.brush_radius(), 0))
                this->app_.brush_radius() = std::clamp(this->app_.brush_radius(), 0, 5);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("Strength", &this->app_.brush_strength_, 0.01f, 1.0f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(56);
            if (ImGui::InputFloat("##st_val", &this->app_.brush_strength_, 0.0f, 0.0f, "%.2f"))
                this->app_.brush_strength_ = std::clamp(this->app_.brush_strength_, 0.01f, 1.0f);
            break;
        }
        case EditorTool::FlattenTerrain:
            ImGui::Text("Brush:");
            ImGui::SameLine();
            ImGui::SliderInt("##br", &this->app_.brush_radius(), 0, 5);
            break;
        case EditorTool::PaintTerrain: {
            ImGui::Text("Brush:");
            ImGui::SameLine();
            ImGui::SliderInt("##br", &this->app_.brush_radius(), 0, 5);
            ImGui::SameLine();
            const char* types[] = {"Grass", "Dirt", "Rock", "Sand", "Snow"};
            int cur = (int)this->app_.paint_type_;
            if (ImGui::Combo("##pt", &cur, types, 5)) this->app_.paint_type_ = (TerrainType)cur;
            break;
        }
    }
    ImGui::PopStyleVar();
    ImGui::End();
}

void EditorPanels::render_palette() {
    ImGui::Begin("Palette", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);
    auto pick = [this](const char* label, PlaceableType type) {
        bool active = this->app_.placeable_ == type;
        if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.5f, 0.75f, 1.0f));
        if (ImGui::Button(label, ImVec2(-1, 0))) this->app_.placeable_ = type;
        if (active) ImGui::PopStyleColor();
    };
    if (ImGui::CollapsingHeader("Animals")) {
        pick("Cow", PlaceableType::Cow);
        pick("Horse", PlaceableType::Horse);
        pick("Llama", PlaceableType::Llama);
        pick("Pig", PlaceableType::Pig);
        pick("Pug", PlaceableType::Pug);
        pick("Sheep", PlaceableType::Sheep);
        pick("Zebra", PlaceableType::Zebra);
    }
    if (ImGui::CollapsingHeader("Buildings")) {
        pick("Floor", PlaceableType::Floor);
        pick("Wall", PlaceableType::Wall);
    }
    if (ImGui::CollapsingHeader("Furniture")) {
        pick("Bed", PlaceableType::Bed);
        pick("Chair", PlaceableType::Chair);
        pick("Couch", PlaceableType::Couch);
        pick("Light Floor", PlaceableType::LightFloor);
        pick("Shelf", PlaceableType::Shelf);
        pick("Table", PlaceableType::Table);
    }
    if (ImGui::CollapsingHeader("Lights")) {
        pick("Point Light", PlaceableType::PointLight);
    }
    if (ImGui::CollapsingHeader("Nature")) {
        pick("Bush", PlaceableType::Bush);
        pick("Bush 2", PlaceableType::Bush2);
        pick("Dead 1", PlaceableType::Dead1);
        pick("Dead 2", PlaceableType::Dead2);
        pick("Dead 3", PlaceableType::Dead3);
        pick("Dead 4", PlaceableType::Dead4);
        pick("Dead 5", PlaceableType::Dead5);
        pick("Pine 1", PlaceableType::Pine1);
        pick("Pine 2", PlaceableType::Pine2);
        pick("Pine 3", PlaceableType::Pine3);
        pick("Pine 4", PlaceableType::Pine4);
        pick("Pine 5", PlaceableType::Pine5);
        pick("Rock 1", PlaceableType::Rock1);
        pick("Rock 2", PlaceableType::Rock2);
        pick("Rock 3", PlaceableType::Rock3);
        pick("Rock 4", PlaceableType::Rock4);
        pick("Rock Large 1", PlaceableType::RockLarge1);
        pick("Rock Large 2", PlaceableType::RockLarge2);
        pick("Rock Large 3", PlaceableType::RockLarge3);
        pick("Tree", PlaceableType::Tree);
        pick("Tree 2", PlaceableType::Tree2);
        pick("Tree 3", PlaceableType::Tree3);
        pick("Tree 4", PlaceableType::Tree4);
        pick("Tree 5", PlaceableType::Tree5);
        pick("Twisted 1", PlaceableType::Twisted1);
        pick("Twisted 2", PlaceableType::Twisted2);
        pick("Twisted 3", PlaceableType::Twisted3);
        pick("Twisted 4", PlaceableType::Twisted4);
        pick("Twisted 5", PlaceableType::Twisted5);
    }
    if (ImGui::CollapsingHeader("Robots")) {
        pick("George", PlaceableType::RobotGeorge);
        pick("Leela", PlaceableType::RobotLeela);
        pick("Mike", PlaceableType::RobotMike);
        pick("Stan", PlaceableType::RobotStan);
    }
    if (ImGui::CollapsingHeader("Space")) {
        pick("Geodesic Dome", PlaceableType::GeodesicDome);
        pick("Moon Rock", PlaceableType::MoonRock);
        pick("Moon Rock 2", PlaceableType::MoonRock2);
        pick("Moon Rock 3", PlaceableType::MoonRock3);
        pick("Moon Rock Large", PlaceableType::MoonRockLarge);
        pick("Solar Panel", PlaceableType::SolarPanel);
        pick("Space Base", PlaceableType::SpaceBase);
    }
    if (this->app_.placeable_ == PlaceableType::Wall)
        ImGui::TextWrapped("Drag to draw a wall run. Click for a single wall.");
    else if (this->app_.placeable_ == PlaceableType::Floor)
        ImGui::TextWrapped("Drag from one corner to the opposite to fill a floor.");
    ImGui::End();
}

void EditorPanels::render_hierarchy() {
    ImGui::Begin("Hierarchy", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);
    size_t n = this->app_.world_.entities().max_allocated();
    std::vector<Entity> rob, obj;
    for (size_t i = 1; i <= n; ++i) {
        Entity e = (Entity)i;
        if (!this->app_.world_.valid(e)) continue;
        if (!this->app_.world_.get_component<Transform3D>(e)) continue;
        if (this->app_.world_.has_component<DifferentialDrive>(e))
            rob.push_back(e);
        else
            obj.push_back(e);
    }
    if (ImGui::TreeNode("Objects")) {
        for (auto e : obj) {
            auto* nn = this->app_.world_.get_component<Name>(e);
            std::string l = nn ? nn->value : "obj_" + std::to_string(e);
            auto f = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
            if (this->app_.placement_.is_selected(e)) f |= ImGuiTreeNodeFlags_Selected;
            ImGui::TreeNodeEx(l.c_str(), f);
            if (ImGui::IsItemClicked()) {
                if (ImGui::GetIO().KeyCtrl)
                    this->app_.placement_.toggle_selection(e);
                else {
                    this->app_.placement_.clear_selection();
                    this->app_.placement_.add_selection(e);
                }
            }
        }
        ImGui::TreePop();
    }
    if (ImGui::TreeNode("Robots")) {
        for (auto e : rob) {
            auto* nn = this->app_.world_.get_component<Name>(e);
            std::string l = nn ? nn->value : "bot_" + std::to_string(e);
            auto f = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
            if (this->app_.placement_.is_selected(e)) f |= ImGuiTreeNodeFlags_Selected;
            ImGui::TreeNodeEx(l.c_str(), f);
            if (ImGui::IsItemClicked()) {
                if (ImGui::GetIO().KeyCtrl)
                    this->app_.placement_.toggle_selection(e);
                else {
                    this->app_.placement_.clear_selection();
                    this->app_.placement_.add_selection(e);
                }
            }
        }
        ImGui::TreePop();
    }
    ImGui::End();
}

}  // namespace robcraft::editor
