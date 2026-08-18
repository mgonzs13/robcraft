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

#include "robcraft/engine/core/app_menu.hpp"

#include <imgui.h>

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

AppMenuResult render_app_menu(AppMenuState& state, int current_texture_size,
                              bool current_world_frame) {
    AppMenuResult result;
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open...", "Ctrl+O")) state.pending = AppAction::Open;
            if (ImGui::MenuItem("Reset", "Ctrl+R")) state.pending = AppAction::Reset;
            ImGui::Separator();
            if (ImGui::MenuItem("Options...")) state.options_open = true;
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) state.pending = AppAction::Exit;
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    // The menu bar closed above; surface any deferred action now.
    if (state.pending != AppAction::None && !ImGui::IsPopupOpen("Options")) {
        result.action = state.pending;
        state.pending = AppAction::None;
    }

    if (state.options_open) {
        if (!ImGui::IsPopupOpen("Options")) {
            state.options_index = texture_size_to_index(current_texture_size);
            state.world_frame = current_world_frame;
            ImGui::OpenPopup("Options");
        }
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(
            ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("Options", &state.options_open,
                                   ImGuiWindowFlags_AlwaysAutoResize)) {
            const char* sizes[] = {"256", "512", "1024"};
            ImGui::Combo("Texture size", &state.options_index, sizes, 3);
            ImGui::Checkbox("Publish world frame", &state.world_frame);
            if (ImGui::Button("Apply")) {
                int new_size = index_to_texture_size(state.options_index);
                if (new_size != current_texture_size) result.texture_size = new_size;
                if (state.world_frame != current_world_frame)
                    result.world_frame = state.world_frame ? 1 : 0;
                state.options_open = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                state.options_open = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }
    return result;
}

}  // namespace robcraft::engine::core
