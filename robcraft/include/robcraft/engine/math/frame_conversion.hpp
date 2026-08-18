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

#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

/**
 * @brief Maps a sim position (Y-up/Z-forward) into REP-103 (X-forward/Z-up).
 * @param sim_pos Position in sim world coordinates.
 * @return The equivalent position in REP-103 coordinates.
 */
Vec3 sim_to_rep103_position(const Vec3& sim_pos);

/**
 * @brief Maps a sim orientation (Y-up/Z-forward) into REP-103 (X-forward/Z-up).
 * @param sim_rot Orientation in sim world coordinates.
 * @return The equivalent orientation in REP-103 coordinates.
 */
Quaternion sim_to_rep103_orientation(const Quaternion& sim_rot);

}  // namespace robcraft::engine::math
