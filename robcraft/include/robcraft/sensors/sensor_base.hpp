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

namespace robcraft::sensors {

using namespace robcraft::engine::math;

/** @brief Base configuration shared by all simulated sensor models. */
struct SensorBase {
    /** @brief Update rate in Hz. */
    double update_rate = 10.0;
    /** @brief Time accumulated since the last update. */
    double time_since_update = 0.0;
    /** @brief Mount position offset from base_link in sim coordinates. */
    Vec3 position{0.0, 0.0, 0.0};
    /** @brief Mount rotation offset from base_link as euler angles in radians. */
    Vec3 rotation{0.0, 0.0, 0.0};
};

}  // namespace robcraft::sensors
