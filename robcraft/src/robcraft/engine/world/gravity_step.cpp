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

#include "robcraft/engine/world/gravity_step.hpp"

namespace robcraft::engine::world {

void apply_gravity(double& y, double& vy, double rest_y, double gravity, double dt) {
    if (y <= rest_y && vy <= 0.0) {
        y = rest_y;
        vy = 0.0;
        return;
    }
    vy -= gravity * dt;
    y += vy * dt;
    if (y <= rest_y) {
        y = rest_y;
        vy = 0.0;
    }
}

}  // namespace robcraft::engine::world
