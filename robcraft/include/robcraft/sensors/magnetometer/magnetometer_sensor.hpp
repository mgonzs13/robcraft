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
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/sensors/sensor_base.hpp"

namespace robcraft::sensors::magnetometer {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::sensors;

/** @brief Magnetometer sensor model with configurable world field and bias drift. */
struct MagnetometerSensor : SensorBase {
    /** @brief World magnetic field strength in microteslas. */
    double field_strength = 50.0;
    /** @brief Magnetic declination in degrees, positive east of north. */
    double declination_deg = 0.0;
    /** @brief Magnetic inclination in degrees, positive downward. */
    double inclination_deg = 65.0;

    /** @brief Standard deviation of field noise in microteslas. */
    double magnetic_field_noise_stddev = 0.0;
    /** @brief Current hard-iron bias in microteslas. */
    Vec3 bias{0.0, 0.0, 0.0};
    /** @brief Rate at which the bias drifts over time. */
    double bias_drift_rate = 0.0;

    /** @brief Last measured field in the body frame, in microteslas. */
    Vec3 magnetic_field{0.0, 0.0, 0.0};

    /** @brief Constructs a magnetometer with its default mount offset. */
    MagnetometerSensor() {
        this->update_rate = 100.0;
        this->position = Vec3(0.0, 0.1, 0.0);
    }
};

/**
 * @brief Updates the magnetometer reading from the robot's world rotation.
 * @param sensor The sensor to update.
 * @param world_rotation The robot's rotation quaternion.
 * @param dt Time step in seconds.
 * @param rng Random source for sensor noise.
 */
void magnetometer_update(MagnetometerSensor& sensor, const Quaternion& world_rotation, double dt,
                         Random& rng);

}  // namespace robcraft::sensors::magnetometer
