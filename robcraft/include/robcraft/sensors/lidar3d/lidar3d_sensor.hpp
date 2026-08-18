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

#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/core/random.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/sensors/sensor_base.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::sensors::lidar3d {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;
using namespace robcraft::sensors;

/** @brief 3D LiDAR sensor model with configurable beams and ray counts. */
struct LidarSensor3D : SensorBase {
    /** @brief Minimum measurable range in meters. */
    double range_min = 0.05;
    /** @brief Maximum measurable range in meters. */
    double range_max = 30.0;
    /** @brief Minimum azimuth angle in radians (full sweep by default). */
    double horizontal_fov_min = robcraft::engine::math::deg_to_rad(-180.0);
    /** @brief Maximum azimuth angle in radians. */
    double horizontal_fov_max = robcraft::engine::math::deg_to_rad(180.0);
    /** @brief Minimum elevation angle in radians. */
    double vertical_fov_min = robcraft::engine::math::deg_to_rad(-15.0);
    /** @brief Maximum elevation angle in radians. */
    double vertical_fov_max = robcraft::engine::math::deg_to_rad(15.0);
    /** @brief Number of rays per horizontal sweep. */
    int horizontal_rays = 360;
    /** @brief Number of vertical beams. */
    int vertical_beams = 16;
    /** @brief Gaussian range noise standard deviation in meters. */
    double noise_stddev = 0.0;

    /** @brief Last measured range per (beam, ray), beam-major. */
    std::vector<float> last_ranges;
    /** @brief Local unit ray direction per (beam, ray), beam-major. */
    std::vector<Vec3> last_dirs;

    /** @brief Constructs a 3D LiDAR and precomputes the ray directions. */
    LidarSensor3D();
    /** @brief Recomputes the per-ray directions from the current limits. */
    void rebuild();
};

/**
 * @brief Casts rays against the world (objects and terrain) and updates ranges.
 * @param self_entity The entity carrying the sensor.
 * @param sensor The sensor to update.
 * @param world The world to ray cast against.
 * @param rng Random source for range noise.
 */
void lidar3d_update(Entity self_entity, LidarSensor3D& sensor, World& world, Random& rng);

}  // namespace robcraft::sensors::lidar3d
