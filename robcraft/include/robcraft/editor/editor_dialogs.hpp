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

#include <string>

namespace robcraft::editor {

class EditorApp;
enum class PendingAction;

/** @brief Renders modal dialogs and routes deferred file actions. */
class EditorDialogs {
public:
    /** @brief Constructs the dialogs.
     *  @param app The owning editor application. */
    explicit EditorDialogs(EditorApp& app);

    /** @brief Renders all modal dialogs (path fallback, New World, unsaved changes). */
    void render_dialogs();
    /** @brief Opens a world via the native dialog (or fallback modal). */
    void request_open_world();
    /** @brief Saves the world via the native dialog (or fallback modal).
     *  @return True if a save completed; false on cancel or pending fallback. */
    bool request_save_world();
    /** @brief Renders the text-input fallback modal when zenity is absent. */
    void render_path_fallback();
    /** @brief Opens the New World dialog. */
    void begin_new_world();
    /** @brief Renders the New World modal. */
    void render_new_world_dialog();
    /** @brief Renders the File > Options modal. */
    void render_options_dialog();
    /** @brief Renders the unsaved-changes confirmation modal. */
    void render_unsaved_dialog();
    /** @brief Routes an action through the unsaved-changes guard. */
    void request_pending_action(PendingAction action);
    /** @brief Executes the currently deferred action. */
    void perform_pending_action();

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
