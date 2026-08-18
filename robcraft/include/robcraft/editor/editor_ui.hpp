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

namespace robcraft::editor {

class EditorApp;

/** @brief Renders the editor's main UI shell (menu bar, toolbar, status bar). */
class EditorUi {
public:
    /** @brief Constructs the UI shell.
     *  @param app The owning editor application. */
    explicit EditorUi(EditorApp& app);

    /** @brief Renders the ImGui layout for all panels. */
    void render_ui();
    /** @brief Renders the main menu bar. */
    void render_menu_bar();
    /** @brief Renders the tool buttons grouped by category. */
    void render_toolbar();
    /** @brief Renders the status bar. */
    void render_status_bar();

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
