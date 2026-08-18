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

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/ecs/vertical_motion.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/robot_factory.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;

using Catch::Approx;

TEST_CASE("create_robot assembles the standard robot", "[robot_factory]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    robcraft::engine::core::Entity e =
        robcraft::renderer::create_robot(world, "robot_mike", 4.0, 3.0);

    REQUIRE(world.valid(e));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value ==
            "robot_mike_" + std::to_string(e));
    REQUIRE(world.has_component<robcraft::engine::ecs::Transform3D>(e));
    REQUIRE(world.has_component<robcraft::robots::differential_drive::DifferentialDrive>(e));
    REQUIRE(world.has_component<robcraft::engine::ecs::VerticalMotion>(e));
    REQUIRE(world.has_component<robcraft::engine::collision::BoxCollider>(e));
    REQUIRE(world.has_component<robcraft::sensors::lidar::LidarSensor2D>(e));
    REQUIRE(world.has_component<robcraft::sensors::imu::ImuSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::magnetometer::MagnetometerSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::camera::CameraSensor>(e));

    auto* drive = world.get_component<robcraft::robots::differential_drive::DifferentialDrive>(e);
    REQUIRE(drive->max_linear_speed == Approx(1.5));
    auto* lidar = world.get_component<robcraft::sensors::lidar::LidarSensor2D>(e);
    REQUIRE(lidar->num_rays == 270);
    REQUIRE(lidar->range_max == Approx(10.0));
    REQUIRE(lidar->update_rate == Approx(15.0));

    auto* tf = world.get_component<robcraft::engine::ecs::Transform3D>(e);
    // robot_mike scale is 3.2; no model passed, so ground_frac 0.5 * scale.y = 1.6.
    REQUIRE(tf->position.y == Approx(0.8));
}

TEST_CASE("create_robot can omit the LiDAR", "[robot_factory]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    robcraft::engine::core::Entity e =
        robcraft::renderer::create_robot(world, "robot_leela", 0.0, 0.0, nullptr, false);

    REQUIRE(world.valid(e));
    REQUIRE(!world.has_component<robcraft::sensors::lidar::LidarSensor2D>(e));
    REQUIRE(world.has_component<robcraft::sensors::imu::ImuSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::magnetometer::MagnetometerSensor>(e));
}
