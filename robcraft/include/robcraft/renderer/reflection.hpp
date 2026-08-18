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

#include <array>

#include "robcraft/engine/math/mat4.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

/**
 * @brief Reflects a view matrix about the horizontal plane y = plane_y.
 * @param view The original view matrix.
 * @param plane_y Height of the reflecting plane.
 * @return The mirrored view matrix (world geometry appears reflected).
 */
Mat4 reflected_view(const Mat4& view, float plane_y);

/**
 * @brief Clip plane keeping geometry above y = plane_y - eps.
 * @param plane_y Height of the reflecting plane.
 * @param eps Downward bias so terrain just below the surface is clipped.
 * @return Plane coefficients {a, b, c, d} for a*x + b*y + c*z + d >= 0.
 */
std::array<float, 4> reflection_clip_plane(float plane_y, float eps);

}  // namespace robcraft::renderer
