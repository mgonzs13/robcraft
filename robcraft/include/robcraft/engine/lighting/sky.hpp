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

/** @brief Gradient sky colors for a world (serialized with the .world file). */
struct Sky {
    /** @brief Sky color at the zenith (directly overhead). */
    Vec3 zenith_color{0.4f, 0.5f, 0.7f};
    /** @brief Sky color at the horizon (eye level). */
    Vec3 horizon_color{0.7f, 0.8f, 0.9f};
};

}  // namespace robcraft::engine::lighting
