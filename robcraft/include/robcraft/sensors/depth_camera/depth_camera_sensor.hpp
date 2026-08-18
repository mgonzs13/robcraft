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

#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/sensors/sensor_base.hpp"

namespace robcraft::sensors::depth_camera {

using namespace robcraft::engine::math;
using namespace robcraft::sensors;

/**
 * @brief Converts a raw OpenGL depth buffer value to linear eye-space depth.
 * @param near_plane Near clip distance in meters.
 * @param far_plane Far clip distance in meters.
 * @param z_buffer Raw depth buffer value in [0, 1].
 * @return Linear view-space depth (distance along the optical axis) in meters.
 */
float linearize_depth(float near_plane, float far_plane, float z_buffer);

/** @brief Depth camera sensor model with configurable resolution and FOV. */
struct DepthCameraSensor : SensorBase {
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

    /** @brief Per-pixel linear depth in meters (width * height, rows top-down).
     *          Pixels with no surface within the far plane hold 0 (no-data). */
    std::vector<float> depth_data;

    /** @brief Constructs a depth camera and allocates its depth buffer. */
    DepthCameraSensor() {
        this->update_rate = 30.0;
        this->position = Vec3(0.0, 0.3, 0.0);
        this->depth_data.resize(this->width * this->height, 0.0f);
    }

    /** @brief Reallocates the depth buffer for the current resolution. */
    void rebuild() { this->depth_data.resize(this->width * this->height, 0.0f); }
};

}  // namespace robcraft::sensors::depth_camera
