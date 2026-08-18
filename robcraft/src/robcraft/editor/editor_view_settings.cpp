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

#include "robcraft/editor/editor_view_settings.hpp"

#include <imgui.h>

#include <algorithm>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/lighting/sky.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::math;
using namespace robcraft::engine::lighting;

EditorViewSettings::EditorViewSettings(EditorApp& app) : app_(app) {}

void EditorViewSettings::render_water_panel() {
    if (!this->app_.show_water_panel_) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Water", &this->app_.show_water_panel_,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::InputFloat("Speed", &this->app_.water_speed_, 0.0f, 0.0f, "%.2f"))
        this->app_.water_speed_ = std::clamp(this->app_.water_speed_, 0.0f, 2.0f);
    if (ImGui::InputFloat("Wave Amplitude", &this->app_.water_wave_amp_, 0.0f, 0.0f, "%.2f"))
        this->app_.water_wave_amp_ = std::clamp(this->app_.water_wave_amp_, 0.0f, 3.0f);
    if (ImGui::InputFloat("Specular", &this->app_.water_specular_, 0.0f, 0.0f, "%.2f"))
        this->app_.water_specular_ = std::clamp(this->app_.water_specular_, 0.0f, 2.0f);
    if (ImGui::InputFloat("Opacity", &this->app_.water_opacity_, 0.0f, 0.0f, "%.2f"))
        this->app_.water_opacity_ = std::clamp(this->app_.water_opacity_, 0.0f, 1.0f);
    if (ImGui::InputFloat("Foam", &this->app_.water_foam_, 0.0f, 0.0f, "%.2f"))
        this->app_.water_foam_ = std::clamp(this->app_.water_foam_, 0.0f, 1.5f);
    if (ImGui::InputFloat("Foam Width", &this->app_.water_foam_width_, 0.0f, 0.0f, "%.2f"))
        this->app_.water_foam_width_ = std::clamp(this->app_.water_foam_width_, 0.0f, 4.0f);
    ImGui::Checkbox("Reflection", &this->app_.water_reflection_);
    if (this->app_.water_reflection_)
        if (ImGui::InputFloat("Reflection Strength", &this->app_.water_reflection_strength_, 0.0f,
                              0.0f, "%.2f"))
            this->app_.water_reflection_strength_ =
                std::clamp(this->app_.water_reflection_strength_, 0.0f, 1.0f);
    ImGui::End();
}

void EditorViewSettings::render_lighting_panel() {
    if (!this->app_.show_lighting_panel_) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Lighting", &this->app_.show_lighting_panel_,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);
    SceneLighting& l = this->app_.world_.lighting();
    auto edit_vec3 = [&](const char* label, Vec3& v) {
        float f[3] = {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
        if (ImGui::InputFloat3(label, f)) {
            v = Vec3(f[0], f[1], f[2]);
            this->app_.tools_.mark_modified();
        }
    };
    edit_vec3("Sun Direction", l.sun_direction);
    edit_vec3("Sun Color", l.sun_color);
    float si = l.sun_intensity;
    if (ImGui::InputFloat("Sun Intensity", &si, 0.0f, 0.0f, "%.2f")) {
        l.sun_intensity = si;
        this->app_.tools_.mark_modified();
    }
    edit_vec3("Ambient Color", l.ambient_color);
    float ai = l.ambient_intensity;
    if (ImGui::InputFloat("Ambient Intensity", &ai, 0.0f, 0.0f, "%.2f")) {
        l.ambient_intensity = ai;
        this->app_.tools_.mark_modified();
    }
    bool sh = l.shadows_enabled;
    if (ImGui::Checkbox("Shadows", &sh)) {
        l.shadows_enabled = sh;
        this->app_.tools_.mark_modified();
    }
    ImGui::End();
}

void EditorViewSettings::render_sky_panel() {
    if (!this->app_.show_sky_panel_) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Sky", &this->app_.show_sky_panel_,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);
    robcraft::engine::lighting::Sky& sky = this->app_.world_.sky();
    float zen[3] = {static_cast<float>(sky.zenith_color.x), static_cast<float>(sky.zenith_color.y),
                    static_cast<float>(sky.zenith_color.z)};
    if (ImGui::ColorEdit3("Zenith Color", zen)) {
        sky.zenith_color = robcraft::engine::math::Vec3(zen[0], zen[1], zen[2]);
        this->app_.tools_.mark_modified();
    }
    float hor[3] = {static_cast<float>(sky.horizon_color.x),
                    static_cast<float>(sky.horizon_color.y),
                    static_cast<float>(sky.horizon_color.z)};
    if (ImGui::ColorEdit3("Horizon Color", hor)) {
        sky.horizon_color = robcraft::engine::math::Vec3(hor[0], hor[1], hor[2]);
        this->app_.tools_.mark_modified();
    }
    ImGui::End();
}

void EditorViewSettings::render_physics_panel() {
    if (!this->app_.show_physics_panel_) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Physics", &this->app_.show_physics_panel_,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);
    float g = static_cast<float>(this->app_.world_.gravity());
    if (ImGui::InputFloat("Gravity (m/s^2)", &g, 0.0f, 0.0f, "%.2f")) {
        this->app_.world_.set_gravity(std::clamp(static_cast<double>(g), 0.0, 50.0));
        this->app_.tools_.mark_modified();
    }
    ImGui::End();
}

}  // namespace robcraft::editor
