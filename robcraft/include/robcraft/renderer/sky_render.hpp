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

#include "robcraft/engine/lighting/sky.hpp"
#include "robcraft/engine/math/mat4.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;

class Shader;

/**
 * @brief Draws the gradient sky background into the currently bound FBO.
 *
 * Sets uModel/uView/uProjection to identity; the caller must re-bind its
 * camera matrices and uUseClipPlane/uHasSkin afterwards.
 * @param shader The default shader program (must be bound before calling).
 * @param proj The active camera projection matrix.
 * @param sky The world sky settings.
 * @param mirror_y When true, flips the vertical gradient (for mirrored
 * water-reflection cameras, where view-space +Y points world-down).
 */
void draw_sky_background(Shader& shader, const Mat4& proj, const Sky& sky, bool mirror_y = false);

}  // namespace robcraft::renderer
