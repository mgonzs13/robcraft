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

#include "robcraft/engine/core/texture_size.hpp"

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/** @brief Actions the main-app menu can request. */
enum class AppAction { None, Open, Reset, Exit };

/** @brief Persistent UI state for the main-app menu. */
struct AppMenuState {
    /** @brief Whether the Options modal is open. */
    bool options_open = false;
    /** @brief Selected texture-size combo index (0=256, 1=512, 2=1024). */
    int options_index = 0;
    /** @brief Whether the world (map) TF frame is published. */
    bool world_frame = false;
    /** @brief Deferred menu action, consumed after the menu bar closes. */
    AppAction pending = AppAction::None;
};

/** @brief Actions requested by the menu this frame. */
struct AppMenuResult {
    /** @brief Action requested this frame, or AppAction::None. */
    AppAction action = AppAction::None;
    /** @brief New texture size to apply, or 0 if unchanged. */
    int texture_size = 0;
    /** @brief New world-frame toggle to apply, or -1 if unchanged (0 = off, 1 = on). */
    int world_frame = -1;
};

/**
 * @brief Renders the main-app menu bar and Options modal for one frame.
 * @param state Persistent menu UI state.
 * @param current_texture_size Current texture size to seed the combo.
 * @param current_world_frame Current world-frame toggle to seed the checkbox.
 * @return Actions requested this frame (caller acts on them after the call).
 */
AppMenuResult render_app_menu(AppMenuState& state, int current_texture_size,
                              bool current_world_frame);

}  // namespace robcraft::engine::core
