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

#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

#include <cmath>

#include "robcraft/engine/math/constants.hpp"

namespace robcraft::sensors::magnetometer {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

void magnetometer_update(MagnetometerSensor& sensor, const Quaternion& world_rotation, double dt,
                         Random& rng) {
    const double inc = deg_to_rad(sensor.inclination_deg);
    const double dec = deg_to_rad(sensor.declination_deg);

    // World field in sim coordinates (Y-up, Z-forward): north = +X, east = +Z,
    // down = -Y — the same convention the GPS model uses (GPS maps sim +X to
    // latitude and sim +Z to longitude, matching the REP-103 odom frame).
    // Positive declination rotates the horizontal component east of north;
    // inclination adds the downward component.
    Vec3 b_world(sensor.field_strength * (std::cos(inc) * std::cos(dec)),
                 sensor.field_strength * (-std::sin(inc)),
                 sensor.field_strength * (std::cos(inc) * std::sin(dec)));

    // Sensor world rotation = robot rotation composed with the mount rotation
    // (matches the lidar ray transform and the base_link -> mag_link TF).
    Quaternion sensor_rot =
        Quaternion::from_euler(sensor.rotation.x, sensor.rotation.y, sensor.rotation.z);
    Quaternion total = world_rotation * sensor_rot;

    Vec3 body = total.conjugate().rotate(b_world);

    body.x += rng.gaussian(0.0, sensor.magnetic_field_noise_stddev);
    body.y += rng.gaussian(0.0, sensor.magnetic_field_noise_stddev);
    body.z += rng.gaussian(0.0, sensor.magnetic_field_noise_stddev);

    sensor.bias.x += rng.gaussian(0.0, sensor.bias_drift_rate * dt);
    sensor.bias.y += rng.gaussian(0.0, sensor.bias_drift_rate * dt);
    sensor.bias.z += rng.gaussian(0.0, sensor.bias_drift_rate * dt);

    sensor.magnetic_field = body + sensor.bias;
    sensor.time_since_update = 0.0;
}

}  // namespace robcraft::sensors::magnetometer
