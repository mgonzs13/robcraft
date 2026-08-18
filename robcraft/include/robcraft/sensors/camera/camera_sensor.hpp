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

#include <vector>

#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/fbo.hpp"
#include "robcraft/sensors/sensor_base.hpp"

namespace robcraft::sensors::camera {

using namespace robcraft::engine::math;
using namespace robcraft::sensors;

/** @brief RGB camera sensor model with configurable resolution and FOV. */
struct CameraSensor : SensorBase {
    /** @brief Image width in pixels. */
    int width = 640;
    /** @brief Image height in pixels. */
    int height = 480;
    /** @brief Vertical field of view in degrees. */
    double fov_deg = 60.0;
    /** @brief Near clip plane distance. */
    double near_plane = 0.1;
    /** @brief Far clip plane distance. */
    double far_plane = 100.0;

    /** @brief Raw RGB image data (width * height * 3 bytes). */
    std::vector<uint8_t> image_data;

    /** @brief Constructs a camera and allocates its image buffer. */
    CameraSensor() {
        this->update_rate = 30.0;
        this->position = Vec3(0.0, 0.3, 0.0);
        this->image_data.resize(this->width * this->height * 3, 0);
    }

    /** @brief Reallocates the image buffer for the current resolution. */
    void rebuild() { this->image_data.resize(this->width * this->height * 3, 0); }
};

}  // namespace robcraft::sensors::camera
