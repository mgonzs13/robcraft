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

namespace robcraft::sensors::imu {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::sensors;

/** @brief IMU sensor model with configurable noise and gyro/accel bias drift. */
struct ImuSensor : SensorBase {
    /** @brief Standard deviation of angular velocity noise. */
    double angular_velocity_noise_stddev = 0.0;
    /** @brief Standard deviation of linear acceleration noise. */
    double linear_acceleration_noise_stddev = 0.0;

    /** @brief Current gyroscope bias. */
    Vec3 gyro_bias{0.0, 0.0, 0.0};
    /** @brief Current accelerometer bias. */
    Vec3 accel_bias{0.0, 0.0, 0.0};
    /** @brief Rate at which the biases drift over time. */
    double bias_drift_rate = 0.0;

    /** @brief Estimated orientation as a quaternion. */
    Quaternion orientation;
    /** @brief Estimated angular velocity. */
    Vec3 angular_velocity;
    /** @brief Estimated linear acceleration. */
    Vec3 linear_acceleration;

    /** @brief Previous world-space velocity, used to derive acceleration. */
    Vec3 prev_world_velocity{0.0, 0.0, 0.0};
    /** @brief Whether a previous velocity sample is available. */
    bool has_prev_velocity = false;

    /** @brief Constructs an IMU with its default update rate. */
    ImuSensor() { this->update_rate = 100.0; }
};

/**
 * @brief Updates the IMU estimates from the robot's world velocity and rotation.
 * @param sensor The sensor to update.
 * @param world_velocity The robot's world-space velocity.
 * @param rotation The robot's rotation quaternion.
 * @param omega_yaw The yaw angular velocity in rad/s.
 * @param gravity World gravity magnitude in m/s^2 (world Y-axis points up).
 * @param dt Time step in seconds.
 * @param rng Random source for sensor noise.
 */
void imu_update(ImuSensor& sensor, const Vec3& world_velocity, const Quaternion& rotation,
                double omega_yaw, double gravity, double dt, Random& rng);

}  // namespace robcraft::sensors::imu
