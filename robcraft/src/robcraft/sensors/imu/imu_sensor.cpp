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

#include "robcraft/sensors/imu/imu_sensor.hpp"

#include <cmath>

#include "robcraft/engine/core/random.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::sensors::imu {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

void imu_update(ImuSensor& sensor, const Vec3& world_velocity, const Quaternion& rotation,
                double omega_yaw, double gravity, double dt, Random& rng) {
    Quaternion mount_rotation =
        Quaternion::from_euler(sensor.rotation.x, sensor.rotation.y, sensor.rotation.z);
    Quaternion sensor_rotation = rotation * mount_rotation;
    sensor.orientation = sensor_rotation;

    Vec3 sensor_angular_velocity = mount_rotation.conjugate().rotate(Vec3(0.0, omega_yaw, 0.0));

    sensor.angular_velocity =
        Vec3(sensor_angular_velocity.x + rng.gaussian(0.0, sensor.angular_velocity_noise_stddev),
             sensor_angular_velocity.y + rng.gaussian(0.0, sensor.angular_velocity_noise_stddev),
             sensor_angular_velocity.z + rng.gaussian(0.0, sensor.angular_velocity_noise_stddev));

    sensor.gyro_bias.x += rng.gaussian(0.0, sensor.bias_drift_rate * dt);
    sensor.gyro_bias.y += rng.gaussian(0.0, sensor.bias_drift_rate * dt);
    sensor.gyro_bias.z += rng.gaussian(0.0, sensor.bias_drift_rate * dt);
    sensor.angular_velocity = sensor.angular_velocity + sensor.gyro_bias;

    Quaternion inv = sensor_rotation.conjugate().normalized();
    Vec3 accel_sensor;
    if (sensor.has_prev_velocity) {
        Vec3 accel_world = (world_velocity - sensor.prev_world_velocity) / dt;
        accel_sensor = inv.rotate(accel_world);
    }
    // An accelerometer measures specific force: the kinematic acceleration minus
    // gravity. With the world Y-axis up, the gravity vector is (0, -gravity, 0),
    // so the accelerometer reads (0, +gravity, 0) in the sensor frame at rest.
    accel_sensor = accel_sensor + inv.rotate(Vec3(0.0, gravity, 0.0));

    sensor.prev_world_velocity = world_velocity;
    sensor.has_prev_velocity = true;

    sensor.accel_bias.x += rng.gaussian(0.0, sensor.bias_drift_rate * dt * 0.1);
    sensor.accel_bias.y += rng.gaussian(0.0, sensor.bias_drift_rate * dt * 0.1);
    sensor.accel_bias.z += rng.gaussian(0.0, sensor.bias_drift_rate * dt * 0.1);

    sensor.linear_acceleration =
        Vec3(accel_sensor.x + rng.gaussian(0.0, sensor.linear_acceleration_noise_stddev),
             accel_sensor.y + rng.gaussian(0.0, sensor.linear_acceleration_noise_stddev),
             accel_sensor.z + rng.gaussian(0.0, sensor.linear_acceleration_noise_stddev)) +
        sensor.accel_bias;

    sensor.time_since_update = 0.0;
}

}  // namespace robcraft::sensors::imu
