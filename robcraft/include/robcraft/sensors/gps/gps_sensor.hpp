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

#include "robcraft/engine/core/random.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/sensors/sensor_base.hpp"

namespace robcraft::sensors::gps {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::sensors;

/** @brief World geodetic reference used for lat/lon conversion (WGS-84). */
constexpr double EARTH_RADIUS = 6378137.0;

/** @brief GPS sensor model with configurable noise. */
struct GpsSensor : SensorBase {
    /** @brief Origin latitude in degrees. */
    double origin_lat = 0.0;
    /** @brief Origin longitude in degrees. */
    double origin_lon = 0.0;
    /** @brief Origin altitude in meters. */
    double origin_alt = 0.0;

    /** @brief Standard deviation of position noise in meters. */
    double position_noise_stddev = 0.0;

    /** @brief Last reported latitude in degrees. */
    double latitude = 0.0;
    /** @brief Last reported longitude in degrees. */
    double longitude = 0.0;
    /** @brief Last reported altitude in meters. */
    double altitude = 0.0;

    /** @brief Constructs a GPS with its default mount offset. */
    GpsSensor() { this->position = Vec3(0.0, 0.2, 0.0); }
};

/**
 * @brief Updates the sensor's lat/lon/alt from a world position.
 * @param sensor The sensor to update.
 * @param world_position Position in world coordinates.
 * @param rng Random source for position noise.
 */
void gps_update(GpsSensor& sensor, const Vec3& world_position, Random& rng);

}  // namespace robcraft::sensors::gps
