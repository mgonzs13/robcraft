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

namespace robcraft::engine::collision {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::math;

/** @brief An axis-aligned box collider defined by half-extents. */
struct BoxCollider {
    /** @brief Half-extents of the box along each axis. */
    Vec3 half_extents{0.5, 0.5, 0.5};
};

}  // namespace robcraft::engine::collision
