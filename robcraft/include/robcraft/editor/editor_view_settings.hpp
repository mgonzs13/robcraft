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

/** @brief Renders the water, lighting, and sky settings panels. */
class EditorViewSettings {
public:
    /** @brief Constructs the view-settings panels.
     *  @param app The owning editor application. */
    explicit EditorViewSettings(EditorApp& app);

    /** @brief Renders the water settings panel (tunable shader parameters). */
    void render_water_panel();
    /** @brief Renders the lighting settings panel (sun/ambient). */
    void render_lighting_panel();
    /** @brief Renders the sky settings panel (zenith/horizon colors). */
    void render_sky_panel();
    /** @brief Renders the physics settings panel (gravity). */
    void render_physics_panel();

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
