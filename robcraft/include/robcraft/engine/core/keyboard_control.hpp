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

namespace robcraft::engine::core {

/**
 * @file keyboard_control.hpp
 * @brief Maps keyboard state to a differential-drive command with hold-to-move semantics.
 */

/**
 * @brief A differential-drive command produced from the current keyboard state.
 */
struct KeyboardDriveCommand {
    /** @brief Whether the keyboard is commanding the robot this frame. */
    bool active = false;
    /** @brief Target linear velocity (forward positive) in m/s. */
    double linear = 0.0;
    /** @brief Target angular velocity (left positive) in rad/s. */
    double angular = 0.0;
};

/**
 * @brief Translates I/K/J/L/U key state into a drive command.
 *
 * The command follows hold-to-move semantics: it is active while any movement
 * key is held, and also for the single frame right after the last movement key
 * is released, during which it reports a zero command so the robot stops.
 * While the keyboard is idle (no key held), the command is inactive.
 */
class KeyboardDriveControl {
public:
    /**
     * @brief Computes the drive command for the current key states.
     * @param i Whether the forward key is held.
     * @param k Whether the back key is held.
     * @param j Whether the left-turn key is held.
     * @param l Whether the right-turn key is held.
     * @param u Whether the explicit stop key is held.
     * @return The command to apply this frame.
     */
    KeyboardDriveCommand update(bool i, bool k, bool j, bool l, bool u);

private:
    /** @brief Whether any movement key was held on the previous update. */
    bool movement_held_prev_ = false;
};

}  // namespace robcraft::engine::core
