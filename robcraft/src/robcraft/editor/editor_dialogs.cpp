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

#include "robcraft/editor/editor_dialogs.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <filesystem>
#include <string>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/io/file_dialog.hpp"
#include "robcraft/engine/io/text_path_dialog.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::io;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::robots::differential_drive;

EditorDialogs::EditorDialogs(EditorApp& app) : app_(app) {}

void EditorDialogs::render_dialogs() {
    // Deferred pending-action handling. Popups opened from the File menu or the
    // unsaved-changes dialog are queued by ImGui and would be popped again if
    // another popup is still current, so actions only run (or the save prompt
    // only opens) here, on a frame where no modal is blocking.
    if (this->app_.pending_action_ != PendingAction::None && !this->app_.fallback_open_ &&
        !ImGui::IsPopupOpen("Save changes?")) {
        if (this->app_.world_modified_ && !this->app_.save_prompt_answered_) {
            this->app_.save_prompt_answered_ = true;
            ImGui::OpenPopup("Save changes?");
        } else {
            this->app_.save_prompt_answered_ = false;
            this->perform_pending_action();
        }
    }
    this->render_path_fallback();
    this->render_new_world_dialog();
    this->render_unsaved_dialog();
    this->render_options_dialog();
}

void EditorDialogs::request_pending_action(PendingAction action) {
    // Defer everything to render_dialogs(): calling ImGui::OpenPopup while the
    // File menu popup is still current would pop the new popup immediately.
    this->app_.pending_action_ = action;
    this->app_.save_prompt_answered_ = false;
}

void EditorDialogs::perform_pending_action() {
    switch (this->app_.pending_action_) {
        case PendingAction::NewWorld:
            this->app_.pending_action_ = PendingAction::None;
            this->begin_new_world();
            break;
        case PendingAction::OpenWorld:
            this->app_.pending_action_ = PendingAction::None;
            this->request_open_world();
            break;
        case PendingAction::Exit:
            this->app_.pending_action_ = PendingAction::None;
            glfwSetWindowShouldClose(this->app_.window_, GLFW_TRUE);
            break;
        case PendingAction::None:
            break;
    }
}

void EditorDialogs::begin_new_world() {
    this->app_.new_world_w_ = 32;
    this->app_.new_world_h_ = 32;
    this->app_.new_world_cell_ = 1.0f;
    ImGui::OpenPopup("New World");
}

void EditorDialogs::request_open_world() {
    if (robcraft::engine::io::native_dialog_available()) {
        auto path = robcraft::engine::io::native_open_file();
        if (path) this->app_.tools_.open_world(*path);
        return;
    }
    this->app_.fallback_mode_ = FileDialogMode::Open;
    this->app_.fallback_open_ = true;
}

bool EditorDialogs::request_save_world() {
    if (robcraft::engine::io::native_dialog_available()) {
        auto path = robcraft::engine::io::native_save_file();
        if (!path) return false;
        std::string name = *path;
        if (std::filesystem::path(name).extension() != ".world") name += ".world";
        return this->app_.tools_.save_world(name);
    }
    this->app_.fallback_mode_ = FileDialogMode::SaveAs;
    this->app_.fallback_open_ = true;
    return false;
}

void EditorDialogs::render_path_fallback() {
    if (!this->app_.fallback_open_) return;
    const char* title =
        this->app_.fallback_mode_ == FileDialogMode::Open ? "Open World" : "Save World As";
    auto path = robcraft::engine::io::imgui_path_modal(title, this->app_.fallback_open_);
    if (this->app_.fallback_open_) return;  // modal still open
    if (!path) {                            // cancelled
        this->app_.pending_action_ = PendingAction::None;
        return;
    }
    if (path->empty()) {
        this->app_.fallback_open_ = true;
        return;
    }
    bool done = false;
    if (this->app_.fallback_mode_ == FileDialogMode::Open) {
        done = this->app_.tools_.open_world(*path);
    } else {
        std::string name = *path;
        if (std::filesystem::path(name).extension() != ".world") name += ".world";
        done = this->app_.tools_.save_world(name);
    }
    if (done) {
        if (this->app_.pending_action_ != PendingAction::None) this->perform_pending_action();
    } else {
        this->app_.fallback_open_ = true;  // keep modal open; buffer retains the typed path
    }
}

void EditorDialogs::render_new_world_dialog() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("New World", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;

    ImGui::DragInt("Width (cells)", &this->app_.new_world_w_, 1, 1, 512);
    ImGui::DragInt("Height (cells)", &this->app_.new_world_h_, 1, 1, 512);
    ImGui::DragFloat("Cell size (m)", &this->app_.new_world_cell_, 0.1f, 0.5f, 64.0f);
    if (ImGui::Button("Create")) {
        int w = std::max(this->app_.new_world_w_, 1);
        int h = std::max(this->app_.new_world_h_, 1);
        float cell = std::max(this->app_.new_world_cell_, 0.5f);
        this->app_.tools_.create_world(w, h, cell);
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void EditorDialogs::render_options_dialog() {
    if (!this->app_.show_options_) return;
    // A modal only appears after OpenPopup(); the File menu sets the flag, and
    // render_dialogs (outside the menu-bar frame) opens it here.
    if (!ImGui::IsPopupOpen("Options")) {
        // Seed the pending selection from the current size on each open.
        this->app_.options_texture_index_ = (this->app_.texture_size_ == 512)    ? 1
                                            : (this->app_.texture_size_ == 1024) ? 2
                                                                                 : 0;
        ImGui::OpenPopup("Options");
    }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("Options", &this->app_.show_options_,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;

    const char* sizes[] = {"256", "512", "1024"};
    ImGui::Combo("Texture size", &this->app_.options_texture_index_, sizes, 3);

    if (ImGui::Button("Apply")) {
        int new_size = (this->app_.options_texture_index_ == 1)   ? 512
                       : (this->app_.options_texture_index_ == 2) ? 1024
                                                                  : 256;
        if (new_size != this->app_.texture_size_) {
            this->app_.texture_size_ = new_size;
            this->app_.texture_pack_.load(this->app_.texture_size_);
        }
        this->app_.show_options_ = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        this->app_.show_options_ = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

void EditorDialogs::render_unsaved_dialog() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("Save changes?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::Text("The current world has unsaved changes.");
    ImGui::Text("Save before continuing?");
    if (ImGui::Button("Save")) {
        bool saved = false;
        if (this->app_.current_file_.empty()) {
            saved = this->request_save_world();
            if (!saved && !this->app_.fallback_open_)
                this->app_.pending_action_ = PendingAction::None;
        } else {
            saved = this->app_.tools_.save_world(this->app_.current_file_);
        }
        // The pending action (New World / Open / Exit) runs next frame from
        // render_dialogs once this modal is closed — opening a new popup while
        // a modal is current would pop the new popup right away.
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Discard")) {
        // Do NOT clear world_modified_ here: if the pending action is later
        // cancelled (e.g. New World -> Cancel), the world still has unsaved
        // changes and the save prompt must appear again on the next request.
        // create_world()/open_world() reset world_modified_ only on success.
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        this->app_.pending_action_ = PendingAction::None;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

}  // namespace robcraft::editor
