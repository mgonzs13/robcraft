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

#include "robcraft/engine/io/text_path_dialog.hpp"

#include <imgui.h>

#include <cstring>

namespace robcraft::engine::io {

using namespace robcraft::engine::io;

std::optional<std::string> imgui_path_modal(const char* title, bool& open) {
    if (!open) return std::nullopt;
    static char path_buf[512] = {};
    ImGui::OpenPopup(title);
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        return std::nullopt;
    }
    bool submitted =
        ImGui::InputText("Path", path_buf, sizeof(path_buf), ImGuiInputTextFlags_EnterReturnsTrue);
    std::optional<std::string> result;
    if (ImGui::Button("OK") || (submitted && path_buf[0] != 0)) {
        result = std::string(path_buf);
        open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        path_buf[0] = 0;
        open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
    return result;
}

}  // namespace robcraft::engine::io
