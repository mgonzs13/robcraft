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

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using namespace robcraft::sensors::lidar3d;

using Catch::Approx;

TEST_CASE("LidarSensor3D default configuration", "[lidar3d]") {
    robcraft::sensors::lidar3d::LidarSensor3D sensor;
    REQUIRE(sensor.horizontal_rays == 360);
    REQUIRE(sensor.vertical_beams == 16);
    REQUIRE(sensor.last_ranges.size() == static_cast<size_t>(360) * 16);
    REQUIRE(sensor.last_dirs.size() == static_cast<size_t>(360) * 16);
    REQUIRE(sensor.noise_stddev == Approx(0.0));
    REQUIRE(sensor.update_rate == Approx(15.0));
}

TEST_CASE("LidarSensor3D initial ranges report no detection", "[lidar3d]") {
    robcraft::sensors::lidar3d::LidarSensor3D sensor;
    REQUIRE(sensor.last_ranges.size() == static_cast<size_t>(360) * 16);
    for (float r : sensor.last_ranges) {
        // Before the first raycast the sensor must not fabricate a finite hit
        // (a phantom shell at range_max would poison the first published cloud).
        REQUIRE(std::isinf(r));
    }
}

TEST_CASE("LidarSensor3D middle ray points along +Z", "[lidar3d]") {
    robcraft::sensors::lidar3d::LidarSensor3D sensor;
    sensor.horizontal_rays = 5;
    sensor.vertical_beams = 1;
    sensor.vertical_fov_min = 0.0;
    sensor.vertical_fov_max = 0.0;
    sensor.rebuild();
    const robcraft::engine::math::Vec3& mid = sensor.last_dirs[2];
    REQUIRE(mid.x == Approx(0.0).margin(1e-6));
    REQUIRE(mid.y == Approx(0.0).margin(1e-6));
    REQUIRE(mid.z == Approx(1.0).margin(1e-6));
}

TEST_CASE("LidarSensor3D hits wall in world", "[lidar3d]") {
    robcraft::engine::world::World world;

    auto robot = world.create_entity();
    robcraft::engine::ecs::Transform3D robot_tf;
    robot_tf.position = robcraft::engine::math::Vec3(0.0, 0.25, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(robot, robot_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        robot,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.3, 0.15, 0.3)});

    auto wall = world.create_entity();
    robcraft::engine::ecs::Transform3D wall_tf;
    wall_tf.position = robcraft::engine::math::Vec3(0.0, 1.0, 3.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(wall, wall_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(2.0, 1.0, 0.2)});

    robcraft::sensors::lidar3d::LidarSensor3D sensor;
    sensor.horizontal_rays = 3;
    sensor.vertical_beams = 1;
    sensor.horizontal_fov_min = -0.01;
    sensor.horizontal_fov_max = 0.01;
    sensor.vertical_fov_min = 0.0;
    sensor.vertical_fov_max = 0.0;
    sensor.range_max = 10.0;
    sensor.noise_stddev = 0.0;
    sensor.rebuild();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar3d::lidar3d_update(robot, sensor, world, rng);

    // Middle ray (azimuth 0, elevation 0) points forward (+Z) and hits the
    // wall's near face at z = 3.0 - 0.2 = 2.8 m.
    REQUIRE(sensor.last_ranges[1] == Approx(2.8).margin(0.01));
}

TEST_CASE("LidarSensor3D downward beam hits flat terrain", "[lidar3d]") {
    robcraft::engine::world::World world;
    world.set_terrain(
        robcraft::engine::world::Terrain(16, 16, 1.0));  // flat, height 0, extent -8..8

    auto robot = world.create_entity();
    robcraft::engine::ecs::Transform3D robot_tf;
    robot_tf.position = robcraft::engine::math::Vec3(0.0, 1.0, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(robot, robot_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        robot, robcraft::engine::collision::BoxCollider{});

    robcraft::sensors::lidar3d::LidarSensor3D sensor;
    sensor.horizontal_rays = 1;
    sensor.vertical_beams = 1;
    sensor.horizontal_fov_min = 0.0;
    sensor.horizontal_fov_max = 0.0;
    sensor.vertical_fov_min = robcraft::engine::math::deg_to_rad(-90.0);
    sensor.vertical_fov_max = robcraft::engine::math::deg_to_rad(-90.0);
    sensor.range_max = 10.0;
    sensor.noise_stddev = 0.0;
    sensor.rebuild();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar3d::lidar3d_update(robot, sensor, world, rng);

    // Sensor is mounted at y = 1.0 + 0.15 = 1.15; straight down hits ground at 1.15 m.
    REQUIRE(sensor.last_ranges[0] == Approx(1.15).margin(0.01));
}

TEST_CASE("LidarSensor3D ray with no obstacle returns infinite range", "[lidar3d]") {
    robcraft::engine::world::World world;

    auto robot = world.create_entity();
    robcraft::engine::ecs::Transform3D robot_tf;
    robot_tf.position = robcraft::engine::math::Vec3(0.0, 0.25, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(robot, robot_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        robot,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.3, 0.15, 0.3)});

    robcraft::sensors::lidar3d::LidarSensor3D sensor;
    sensor.horizontal_rays = 3;
    sensor.vertical_beams = 1;
    sensor.horizontal_fov_min = -0.01;
    sensor.horizontal_fov_max = 0.01;
    sensor.vertical_fov_min = 0.0;
    sensor.vertical_fov_max = 0.0;
    sensor.range_max = 10.0;
    sensor.noise_stddev = 0.0;
    sensor.rebuild();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar3d::lidar3d_update(robot, sensor, world, rng);

    // No wall or terrain in front: every ray must report +inf (no detection).
    for (size_t i = 0; i < sensor.last_ranges.size(); ++i) {
        REQUIRE(std::isinf(sensor.last_ranges[i]));
    }
}

TEST_CASE("LidarSensor3D noise adds variation", "[lidar3d]") {
    robcraft::sensors::lidar3d::LidarSensor3D sensor;
    sensor.horizontal_rays = 50;
    sensor.vertical_beams = 1;
    sensor.horizontal_fov_min = -0.02;
    sensor.horizontal_fov_max = 0.02;
    sensor.vertical_fov_min = 0.0;
    sensor.vertical_fov_max = 0.0;
    sensor.noise_stddev = 0.05;
    sensor.rebuild();

    robcraft::engine::world::World world;
    auto e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(
        e, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(0, 0.25, 0)});
    world.add_component<robcraft::engine::collision::BoxCollider>(
        e, robcraft::engine::collision::BoxCollider{});

    auto wall = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(
        wall, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(0, 1.0, 3.0)});
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(2.0, 1.0, 0.2)});

    robcraft::engine::core::Random rng(99);
    robcraft::sensors::lidar3d::lidar3d_update(e, sensor, world, rng);

    // All 50 rays (FOV ±0.02 rad) hit the wall, so Gaussian noise must vary
    // the (finite) ranges; no ray may report an infinite no-detection range.
    bool has_variation = false;
    for (int i = 1; i < 50; ++i) {
        if (std::abs(sensor.last_ranges[i] - sensor.last_ranges[0]) > 0.001) {
            has_variation = true;
            break;
        }
    }
    REQUIRE(has_variation);
    for (size_t i = 0; i < sensor.last_ranges.size(); ++i) {
        REQUIRE(std::isfinite(sensor.last_ranges[i]));
    }
}
