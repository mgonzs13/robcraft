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

#include "robcraft/sensors/imu/imu_sensor.hpp"

using namespace robcraft::sensors::imu;

using Catch::Approx;

TEST_CASE("IMU default configuration", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;

    REQUIRE(sensor.update_rate == Approx(100.0));
    REQUIRE_FALSE(sensor.has_prev_velocity);
    REQUIRE(sensor.angular_velocity_noise_stddev == Approx(0.0));
    REQUIRE(sensor.linear_acceleration_noise_stddev == Approx(0.0));
    REQUIRE(sensor.bias_drift_rate == Approx(0.0));
}

TEST_CASE("IMU updates orientation from rotation", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    robcraft::engine::core::Random rng(42);

    auto rot = robcraft::engine::math::Quaternion::from_euler(0.0, 0.5, 0.0);
    sensor.has_prev_velocity = true;

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(1.0, 0.0, 0.0), rot,
                                       1.0, 9.81, 0.01, rng);

    REQUIRE(std::abs(sensor.orientation.w - rot.w) < 0.001);
}

TEST_CASE("IMU computes angular velocity", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    sensor.angular_velocity_noise_stddev = 0.0;
    sensor.bias_drift_rate = 0.0;
    robcraft::engine::core::Random rng(42);

    auto rot = robcraft::engine::math::Quaternion::identity();
    sensor.has_prev_velocity = true;

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(1.0, 0.0, 0.0), rot,
                                       2.5, 9.81, 0.01, rng);

    REQUIRE(sensor.angular_velocity.y == Approx(2.5));
}

TEST_CASE("IMU noise produces variation", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    sensor.angular_velocity_noise_stddev = 0.1;
    sensor.linear_acceleration_noise_stddev = 0.2;
    robcraft::engine::core::Random rng(123);

    auto rot = robcraft::engine::math::Quaternion::identity();
    sensor.has_prev_velocity = true;

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(1.0, 0.0, 0.0), rot,
                                       0.0, 9.81, 0.01, rng);

    bool has_noise = (std::abs(sensor.angular_velocity.y) > 0.001) ||
                     (std::abs(sensor.linear_acceleration.x) > 0.001);
    REQUIRE(has_noise);
}

TEST_CASE("IMU includes gravity for a resting robot", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    robcraft::engine::core::Random rng(42);

    auto rot = robcraft::engine::math::Quaternion::identity();
    sensor.has_prev_velocity = true;

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(0.0, 0.0, 0.0), rot,
                                       0.0, 9.81, 0.01, rng);

    REQUIRE(sensor.linear_acceleration.x == Approx(0.0));
    REQUIRE(sensor.linear_acceleration.y == Approx(9.81));
    REQUIRE(sensor.linear_acceleration.z == Approx(0.0));
}

TEST_CASE("IMU gravity rotates with the robot attitude", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    robcraft::engine::core::Random rng(7);

    auto rot = robcraft::engine::math::Quaternion::from_euler(3.14159265358979 / 2.0, 0.0, 0.0);
    sensor.has_prev_velocity = true;

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(0.0, 0.0, 0.0), rot,
                                       0.0, 9.81, 0.01, rng);

    REQUIRE(sensor.linear_acceleration.y == Approx(0.0).margin(1e-12));
    REQUIRE(sensor.linear_acceleration.z == Approx(-9.81));
}

TEST_CASE("IMU reports measurements in a rotated mount frame", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    sensor.rotation = robcraft::engine::math::Vec3(3.14159265358979 / 2.0, 0.0, 0.0);
    sensor.has_prev_velocity = true;
    robcraft::engine::core::Random rng(11);

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(0.0, 0.0, 0.0),
                                       robcraft::engine::math::Quaternion::identity(), 0.0, 9.81,
                                       0.01, rng);

    REQUIRE(sensor.linear_acceleration.x == Approx(0.0));
    REQUIRE(sensor.linear_acceleration.y == Approx(0.0).margin(1e-12));
    REQUIRE(sensor.linear_acceleration.z == Approx(-9.81));
    REQUIRE(sensor.angular_velocity.x == Approx(0.0));
    REQUIRE(sensor.angular_velocity.y == Approx(0.0).margin(1e-12));
    REQUIRE(sensor.angular_velocity.z == Approx(0.0));

    auto expected =
        robcraft::engine::math::Quaternion::from_euler(3.14159265358979 / 2.0, 0.0, 0.0);
    REQUIRE(sensor.orientation.w == Approx(expected.w));
    REQUIRE(sensor.orientation.x == Approx(expected.x));
    REQUIRE(sensor.orientation.y == Approx(expected.y));
    REQUIRE(sensor.orientation.z == Approx(expected.z));
}

TEST_CASE("IMU primes velocity before deriving first acceleration", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    robcraft::engine::core::Random rng(12);

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(1.0, 0.0, 0.0),
                                       robcraft::engine::math::Quaternion::identity(), 0.0, 9.81,
                                       0.01, rng);

    REQUIRE(sensor.linear_acceleration.x == Approx(0.0));
    REQUIRE(sensor.linear_acceleration.y == Approx(9.81));
    REQUIRE(sensor.linear_acceleration.z == Approx(0.0));
}

TEST_CASE("IMU accelerometer combines kinematic acceleration with gravity", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    robcraft::engine::core::Random rng(9);

    auto rot = robcraft::engine::math::Quaternion::identity();
    sensor.has_prev_velocity = true;

    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(0.0, 0.0, 0.0), rot,
                                       0.0, 9.81, 0.01, rng);
    robcraft::sensors::imu::imu_update(sensor, robcraft::engine::math::Vec3(1.0, 0.0, 0.0), rot,
                                       0.0, 9.81, 0.01, rng);

    REQUIRE(sensor.linear_acceleration.x == Approx(100.0));
    REQUIRE(sensor.linear_acceleration.y == Approx(9.81));
    REQUIRE(sensor.linear_acceleration.z == Approx(0.0));
}

TEST_CASE("IMU time accumulator", "[imu]") {
    robcraft::sensors::imu::ImuSensor sensor;
    sensor.update_rate = 50.0;
    sensor.time_since_update = 0.0;

    sensor.time_since_update += 0.01;
    REQUIRE(sensor.time_since_update < 1.0 / sensor.update_rate);

    sensor.time_since_update += 0.02;
    REQUIRE(sensor.time_since_update >= 1.0 / sensor.update_rate);
}
