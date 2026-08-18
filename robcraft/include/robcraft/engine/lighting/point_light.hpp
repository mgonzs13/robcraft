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

/** @brief Point light attached to an entity; world position comes from Transform3D. */
struct PointLight {
    /** @brief Light color (RGB). */
    Vec3 color{1.0f, 1.0f, 1.0f};
    /** @brief Light intensity multiplier. */
    float intensity = 1.0f;
    /** @brief Light radius in meters (falls off to zero at this distance). */
    float range = 8.0f;
};

}  // namespace robcraft::engine::lighting
