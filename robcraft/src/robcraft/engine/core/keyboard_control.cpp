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

#include "robcraft/engine/core/keyboard_control.hpp"

namespace robcraft::engine::core {

KeyboardDriveCommand KeyboardDriveControl::update(bool i, bool k, bool j, bool l, bool u) {
    bool movement_held = i || k || j || l;

    KeyboardDriveCommand cmd;
    cmd.active = movement_held || this->movement_held_prev_;
    if (i) cmd.linear = 1.0;
    if (k) cmd.linear = -1.0;
    if (j) cmd.angular = 1.0;
    if (l) cmd.angular = -1.0;

    // The explicit stop key forces both targets to zero while held.
    if (u) {
        cmd.linear = 0.0;
        cmd.angular = 0.0;
        cmd.active = true;
    }

    this->movement_held_prev_ = movement_held;
    return cmd;
}

}  // namespace robcraft::engine::core
