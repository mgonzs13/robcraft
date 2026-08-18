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

#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::lighting {

using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;

/** @brief Sun and ambient lighting for a world (serialized with the .world file). */
struct SceneLighting {
    /** @brief Sun light direction (points toward the sun). */
    Vec3 sun_direction{0.5, 1.0, 0.3};
    /** @brief Sun light color (RGB). */
    Vec3 sun_color{1.0f, 0.95f, 0.85f};
    /** @brief Sun intensity multiplier. */
    float sun_intensity = 1.0f;
    /** @brief Ambient fill color (RGB). */
    Vec3 ambient_color{0.4f, 0.42f, 0.45f};
    /** @brief Ambient intensity multiplier. */
    float ambient_intensity = 1.0f;
    /** @brief Whether sun shadows are rendered. */
    bool shadows_enabled = true;
};

}  // namespace robcraft::engine::lighting
