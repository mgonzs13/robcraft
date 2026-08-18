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

/** @brief Renders tool options, the placeable palette, and the entity hierarchy. */
class EditorPanels {
public:
    /** @brief Constructs the panels.
     *  @param app The owning editor application. */
    explicit EditorPanels(EditorApp& app);

    /** @brief Renders tool-specific options (gizmo mode, brush settings) below the toolbar. */
    void render_tool_options();
    /** @brief Renders the placeable palette window (Place tool only). */
    void render_palette();
    /** @brief Renders the entity hierarchy panel. */
    void render_hierarchy();

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
