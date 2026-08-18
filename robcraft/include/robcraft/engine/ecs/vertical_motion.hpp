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

namespace robcraft::engine::ecs {

/**
 * @brief Vertical (Y-axis) dynamics state used by the gravity model.
 *
 * Header-only POD, like DifferentialDrive. Added to every robot alongside
 * DifferentialDrive; a robot is resting on the ground when its Y position is
 * at rest height and this velocity is zero (or negative).
 */
struct VerticalMotion {
    /** @brief Current vertical velocity in m/s (positive = up). */
    double vertical_velocity = 0.0;
};

}  // namespace robcraft::engine::ecs
