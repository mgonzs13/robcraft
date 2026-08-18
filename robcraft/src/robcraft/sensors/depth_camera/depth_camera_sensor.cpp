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

#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"

#include <cmath>

#include "robcraft/engine/math/mat4.hpp"

namespace robcraft::sensors::depth_camera {

using namespace robcraft::engine::math;

float linearize_depth(float near_plane, float far_plane, float z_buffer) {
    // Invert the standard GL projection (Mat4::perspective): depth buffer value
    // in [0,1] -> NDC z in [-1,1] -> linear eye-space distance. Computed in
    // double so the far-plane endpoint does not overshoot 1.0 in float.
    double z_ndc_signed = 2.0 * static_cast<double>(z_buffer) - 1.0;
    double depth =
        2.0 * static_cast<double>(near_plane) * static_cast<double>(far_plane) /
        (static_cast<double>(far_plane) + static_cast<double>(near_plane) -
         z_ndc_signed * (static_cast<double>(far_plane) - static_cast<double>(near_plane)));
    return static_cast<float>(depth);
}

}  // namespace robcraft::sensors::depth_camera
