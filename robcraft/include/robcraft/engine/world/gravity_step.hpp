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

namespace robcraft::engine::world {

/**
 * @brief Integrates one fixed-timestep tick of vertical motion under gravity.
 * @param y In/out vertical world position (meters).
 * @param vy In/out vertical velocity in m/s (positive = up).
 * @param rest_y Ground height the entity rests on (its resting Y).
 * @param gravity Acceleration magnitude in m/s^2 (must be >= 0).
 * @param dt Fixed timestep in seconds.
 *
 * If the entity is on the ground (y <= rest_y) with no upward velocity it is
 * pinned to rest_y with vy = 0. Otherwise gravity accelerates it downward and
 * a ground crossing on this tick clamps it to rest_y with vy = 0. Deterministic:
 * depends only on its arguments.
 */
void apply_gravity(double& y, double& vy, double rest_y, double gravity, double dt);

}  // namespace robcraft::engine::world
