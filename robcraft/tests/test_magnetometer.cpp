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
#include <cmath>

#include "robcraft/engine/math/constants.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

using namespace robcraft::sensors::magnetometer;

using Catch::Approx;

TEST_CASE("Magnetometer default configuration", "[magnetometer]") {
    robcraft::sensors::magnetometer::MagnetometerSensor sensor;

    REQUIRE(sensor.update_rate == Approx(100.0));
    REQUIRE(sensor.field_strength == Approx(50.0));
    REQUIRE(sensor.declination_deg == Approx(0.0));
    REQUIRE(sensor.inclination_deg == Approx(65.0));
    REQUIRE(sensor.position.x == Approx(0.0));
    REQUIRE(sensor.position.y == Approx(0.1));
    REQUIRE(sensor.position.z == Approx(0.0));
    REQUIRE(sensor.magnetic_field_noise_stddev == Approx(0.0));
    REQUIRE(sensor.bias_drift_rate == Approx(0.0));
}

TEST_CASE("Magnetometer reads north-pointing field at yaw zero", "[magnetometer]") {
    robcraft::sensors::magnetometer::MagnetometerSensor sensor;
    sensor.magnetic_field_noise_stddev = 0.0;
    sensor.bias_drift_rate = 0.0;
    robcraft::engine::core::Random rng(42);

    // Defaults: strength 50 uT, declination 0, inclination 65 deg. World field
    // (Y-up/Z-forward sim, north = +X, east = +Z, matching GPS/ENU) =
    // (50*cos(65), -50*sin(65), 0). A yaw-0 robot faces +Z (east); the field
    // points north (+X, to its left) and down.
    robcraft::sensors::magnetometer::magnetometer_update(
        sensor, robcraft::engine::math::Quaternion::identity(), 0.01, rng);

    REQUIRE(sensor.magnetic_field.x ==
            Approx(50.0 * std::cos(65.0 * robcraft::engine::math::kPi / 180.0)).margin(1e-6));
    REQUIRE(sensor.magnetic_field.y ==
            Approx(-50.0 * std::sin(65.0 * robcraft::engine::math::kPi / 180.0)).margin(1e-6));
    REQUIRE(sensor.magnetic_field.z == Approx(0.0).margin(1e-9));
}

TEST_CASE("Magnetometer reading rotates with yaw", "[magnetometer]") {
    robcraft::sensors::magnetometer::MagnetometerSensor sensor;
    sensor.magnetic_field_noise_stddev = 0.0;
    sensor.bias_drift_rate = 0.0;
    robcraft::engine::core::Random rng(42);

    // 90 deg yaw about +Y turns the nose from +Z (east) to +X (north); the
    // field now points forward/down.
    auto rot =
        robcraft::engine::math::Quaternion::from_euler(0.0, robcraft::engine::math::kPi / 2.0, 0.0);
    robcraft::sensors::magnetometer::magnetometer_update(sensor, rot, 0.01, rng);

    REQUIRE(sensor.magnetic_field.x == Approx(0.0).margin(1e-6));
    REQUIRE(sensor.magnetic_field.y ==
            Approx(-50.0 * std::sin(65.0 * robcraft::engine::math::kPi / 180.0)).margin(1e-6));
    REQUIRE(sensor.magnetic_field.z ==
            Approx(50.0 * std::cos(65.0 * robcraft::engine::math::kPi / 180.0)).margin(1e-6));
}

TEST_CASE("Magnetometer declination rotates the horizontal field", "[magnetometer]") {
    robcraft::sensors::magnetometer::MagnetometerSensor sensor;
    sensor.field_strength = 50.0;
    sensor.declination_deg = 20.0;
    sensor.inclination_deg = 0.0;
    sensor.magnetic_field_noise_stddev = 0.0;
    sensor.bias_drift_rate = 0.0;
    robcraft::engine::core::Random rng(42);

    robcraft::sensors::magnetometer::magnetometer_update(
        sensor, robcraft::engine::math::Quaternion::identity(), 0.01, rng);

    // declination 20 deg east of north: horizontal = 50*( cos20, sin20 ) in the
    // sim (X = north, Z = east) plane.
    REQUIRE(sensor.magnetic_field.x ==
            Approx(50.0 * std::cos(20.0 * robcraft::engine::math::kPi / 180.0)).margin(1e-6));
    REQUIRE(sensor.magnetic_field.y == Approx(0.0).margin(1e-9));
    REQUIRE(sensor.magnetic_field.z ==
            Approx(50.0 * std::sin(20.0 * robcraft::engine::math::kPi / 180.0)).margin(1e-6));
}

TEST_CASE("Magnetometer noise produces variation", "[magnetometer]") {
    robcraft::sensors::magnetometer::MagnetometerSensor sensor;
    sensor.field_strength = 0.0;
    sensor.magnetic_field_noise_stddev = 1.0;
    sensor.bias_drift_rate = 0.0;
    robcraft::engine::core::Random rng(123);

    robcraft::sensors::magnetometer::magnetometer_update(
        sensor, robcraft::engine::math::Quaternion::identity(), 0.01, rng);

    bool has_noise = std::abs(sensor.magnetic_field.x) > 0.001 ||
                     std::abs(sensor.magnetic_field.y) > 0.001 ||
                     std::abs(sensor.magnetic_field.z) > 0.001;
    REQUIRE(has_noise);
}

TEST_CASE("Magnetometer bias drifts over time", "[magnetometer]") {
    robcraft::sensors::magnetometer::MagnetometerSensor sensor;
    sensor.field_strength = 0.0;
    sensor.magnetic_field_noise_stddev = 0.0;
    sensor.bias_drift_rate = 0.5;
    robcraft::engine::core::Random rng(7);

    robcraft::sensors::magnetometer::magnetometer_update(
        sensor, robcraft::engine::math::Quaternion::identity(), 0.01, rng);

    bool bias_changed = sensor.bias.x != 0.0 || sensor.bias.y != 0.0 || sensor.bias.z != 0.0;
    REQUIRE(bias_changed);
}

TEST_CASE("Magnetometer time accumulator", "[magnetometer]") {
    robcraft::sensors::magnetometer::MagnetometerSensor sensor;
    sensor.update_rate = 50.0;
    sensor.time_since_update = 0.0;

    sensor.time_since_update += 0.01;
    REQUIRE(sensor.time_since_update < 1.0 / sensor.update_rate);

    sensor.time_since_update += 0.02;
    REQUIRE(sensor.time_since_update >= 1.0 / sensor.update_rate);
}
