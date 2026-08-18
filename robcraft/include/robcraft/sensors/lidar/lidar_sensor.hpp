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

namespace robcraft::sensors::lidar {

using namespace robcraft::engine::world;
using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::sensors;

/** @brief 2D LiDAR sensor model with configurable ray counts and noise. */
struct LidarSensor2D : SensorBase {
    /** @brief Minimum measurable range in meters. */
    double range_min = 0.05;
    /** @brief Maximum measurable range in meters. */
    double range_max = 20.0;
    /** @brief Minimum scan angle in radians. */
    double angle_min = robcraft::engine::math::deg_to_rad(-135.0);
    /** @brief Maximum scan angle in radians. */
    double angle_max = robcraft::engine::math::deg_to_rad(135.0);
    /** @brief Number of rays per scan. */
    int num_rays = 540;
    /** @brief Gaussian range noise standard deviation in meters. */
    double noise_stddev = 0.0;

    /** @brief Last measured range per ray. */
    std::vector<double> last_ranges;
    /** @brief Precomputed scan angle per ray. */
    std::vector<double> last_angles;

    /** @brief Constructs a LiDAR sensor and precomputes the ray angles. */
    LidarSensor2D();
    /** @brief Recomputes the per-ray scan angles from the current limits. */
    void rebuild_angles();
};

/**
 * @brief Casts rays against the world and updates the sensor's ranges.
 * @param self_entity The entity carrying the sensor.
 * @param sensor The sensor to update.
 * @param world The world to ray cast against.
 * @param rng Random source for range noise.
 */
void lidar_update(Entity self_entity, LidarSensor2D& sensor, World& world, Random& rng);

}  // namespace robcraft::sensors::lidar
