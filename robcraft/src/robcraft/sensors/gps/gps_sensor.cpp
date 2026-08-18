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

#include "robcraft/sensors/gps/gps_sensor.hpp"

#include <cmath>

#include "robcraft/engine/core/random.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/frame_conversion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::sensors::gps {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

void gps_update(GpsSensor& sensor, const Vec3& world_position, Random& rng) {
    double noise_x = rng.gaussian(0.0, sensor.position_noise_stddev);
    double noise_y = rng.gaussian(0.0, sensor.position_noise_stddev);
    double noise_z = rng.gaussian(0.0, sensor.position_noise_stddev);

    // The sim world is Y-up/Z-forward; the ENU GPS frame must match the REP-103
    // odom frame (X east = sim +Z, Y north = sim +X, Z up = sim +Y) so that
    // navsat_transform_node can align GPS with the wheel odometry.
    Vec3 rep = sim_to_rep103_position(world_position);

    // Latitude/longitude are stored in decimal degrees (the NavSatFix wire
    // format); the per-meter conversion yields radians, so it is converted.
    double rad_per_meter_lat = 1.0 / EARTH_RADIUS;
    double rad_per_meter_lon = 1.0 / (EARTH_RADIUS * std::cos(deg_to_rad(sensor.origin_lat)));

    double delta_lat = rad_to_deg((rep.y + noise_x) * rad_per_meter_lat);
    double delta_lon = rad_to_deg((rep.x + noise_y) * rad_per_meter_lon);

    sensor.latitude = sensor.origin_lat + delta_lat;
    sensor.longitude = sensor.origin_lon + delta_lon;
    sensor.altitude = sensor.origin_alt + rep.z + noise_z;

    sensor.time_since_update = 0.0;
}

}  // namespace robcraft::sensors::gps
