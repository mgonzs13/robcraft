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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "robcraft/engine/math/constants.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"

using namespace robcraft::engine::math;
using namespace robcraft::sensors::gps;

using Catch::Approx;

TEST_CASE("GPS default configuration", "[gps]") {
    robcraft::sensors::gps::GpsSensor sensor;

    REQUIRE(sensor.update_rate == Approx(10.0));
    REQUIRE(sensor.origin_lat == Approx(0.0));
    REQUIRE(sensor.origin_lon == Approx(0.0));
    REQUIRE(sensor.position_noise_stddev == Approx(0.0));
}

TEST_CASE("GPS converts position to lat/lon at origin", "[gps]") {
    robcraft::sensors::gps::GpsSensor sensor;
    sensor.position_noise_stddev = 0.0;
    robcraft::engine::core::Random rng(42);

    robcraft::sensors::gps::gps_update(sensor, robcraft::engine::math::Vec3(0.0, 0.0, 0.0), rng);

    REQUIRE(sensor.latitude == Approx(0.0));
    REQUIRE(sensor.longitude == Approx(0.0));
    REQUIRE(sensor.altitude == Approx(0.0));
}

TEST_CASE("GPS converts position to lat/lon with offset", "[gps]") {
    robcraft::sensors::gps::GpsSensor sensor;
    sensor.origin_lat = 0.0;
    sensor.origin_lon = 0.0;
    sensor.origin_alt = 100.0;
    sensor.position_noise_stddev = 0.0;
    robcraft::engine::core::Random rng(42);

    // ENU must match the REP-103 odom frame: sim +X -> North (lat), sim +Z ->
    // East (lon), sim +Y -> altitude. 111320 m is ~1 deg at the equator.
    // Latitude/longitude are stored in decimal degrees.
    robcraft::sensors::gps::gps_update(sensor, robcraft::engine::math::Vec3(111320.0, 50.0, 0.0),
                                       rng);
    REQUIRE(sensor.latitude == Approx(1.0).margin(1e-6));
    REQUIRE(sensor.longitude == Approx(0.0).margin(1e-9));
    REQUIRE(sensor.altitude == Approx(150.0));

    robcraft::sensors::gps::gps_update(sensor, robcraft::engine::math::Vec3(0.0, 50.0, 111320.0),
                                       rng);
    REQUIRE(sensor.latitude == Approx(0.0).margin(1e-9));
    REQUIRE(sensor.longitude == Approx(1.0).margin(1e-6));
    REQUIRE(sensor.altitude == Approx(150.0));
}

TEST_CASE("GPS adds noise to position", "[gps]") {
    robcraft::sensors::gps::GpsSensor sensor;
    sensor.position_noise_stddev = 1.0;
    robcraft::engine::core::Random rng(999);

    double lats[10];
    for (int i = 0; i < 10; ++i) {
        robcraft::sensors::gps::gps_update(sensor, robcraft::engine::math::Vec3(0.0, 0.0, 0.0),
                                           rng);
        lats[i] = sensor.latitude;
    }

    bool has_variation = false;
    for (int i = 1; i < 10; ++i) {
        if (std::abs(lats[i] - lats[0]) > 1e-10) {
            has_variation = true;
            break;
        }
    }
    REQUIRE(has_variation);
}

TEST_CASE("GPS time accumulator", "[gps]") {
    robcraft::sensors::gps::GpsSensor sensor;
    sensor.update_rate = 5.0;
    sensor.time_since_update = 0.0;

    sensor.time_since_update += 0.1;
    REQUIRE(sensor.time_since_update < 1.0 / sensor.update_rate);

    sensor.time_since_update += 0.2;
    REQUIRE(sensor.time_since_update >= 1.0 / sensor.update_rate);
}
