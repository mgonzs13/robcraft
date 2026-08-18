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
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using namespace robcraft::sensors::lidar;

using Catch::Approx;

TEST_CASE("LidarSensor2D default configuration", "[lidar]") {
    robcraft::sensors::lidar::LidarSensor2D sensor;

    REQUIRE(sensor.range_min == Approx(0.05));
    REQUIRE(sensor.range_max == Approx(20.0));
    REQUIRE(sensor.num_rays == 540);
    REQUIRE(sensor.update_rate == Approx(15.0));
    REQUIRE(sensor.last_angles.size() == 540);
    REQUIRE(sensor.last_ranges.size() == 540);
    REQUIRE(sensor.noise_stddev == Approx(0.0));
}

TEST_CASE("LidarSensor2D angle calculation", "[lidar]") {
    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 5;
    sensor.rebuild_angles();

    REQUIRE(sensor.last_angles.size() == 5);

    double step = (sensor.angle_max - sensor.angle_min) / 4.0;
    REQUIRE(sensor.last_angles[0] == Approx(sensor.angle_min));
    REQUIRE(sensor.last_angles[4] == Approx(sensor.angle_max));
    REQUIRE(sensor.last_angles[1] == Approx(sensor.angle_min + step));
}

TEST_CASE("Lidar ray with no obstacle returns infinite range", "[lidar]") {
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
    wall_tf.position = robcraft::engine::math::Vec3(3.0, 1.0, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(wall, wall_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.2, 1.0, 2.0)});

    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 3;
    sensor.range_max = 10.0;
    sensor.angle_min = -0.1;
    sensor.angle_max = 0.1;
    sensor.noise_stddev = 0.0;
    sensor.rebuild_angles();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar::lidar_update(robot, sensor, world, rng);

    REQUIRE(sensor.last_ranges.size() == 3);
    // The wall sits off-axis (+X) while every ray points along +Z, so all rays
    // miss it; a no-detection ray must report +inf, not a finite range_max.
    for (int i = 0; i < 3; ++i) {
        REQUIRE(std::isinf(sensor.last_ranges[i]));
    }
}

TEST_CASE("Lidar angle 0 ray points forward along +Z", "[lidar]") {
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

    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 3;
    sensor.range_max = 10.0;
    sensor.angle_min = -0.01;
    sensor.angle_max = 0.01;
    sensor.noise_stddev = 0.0;
    sensor.rebuild_angles();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar::lidar_update(robot, sensor, world, rng);

    REQUIRE(sensor.last_ranges[1] == Approx(2.8).margin(0.01));
}

TEST_CASE("Lidar scan rotates with robot heading", "[lidar]") {
    robcraft::engine::world::World world;

    auto robot = world.create_entity();
    robcraft::engine::ecs::Transform3D robot_tf;
    robot_tf.position = robcraft::engine::math::Vec3(0.0, 0.25, 0.0);
    robot_tf.rotation = robcraft::engine::math::Quaternion::from_euler(0.0, 1.5707963, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(robot, robot_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        robot,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.3, 0.15, 0.3)});

    auto wall = world.create_entity();
    robcraft::engine::ecs::Transform3D wall_tf;
    wall_tf.position = robcraft::engine::math::Vec3(3.0, 1.0, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(wall, wall_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.2, 1.0, 2.0)});

    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 3;
    sensor.range_max = 10.0;
    sensor.angle_min = -0.01;
    sensor.angle_max = 0.01;
    sensor.noise_stddev = 0.0;
    sensor.rebuild_angles();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar::lidar_update(robot, sensor, world, rng);

    REQUIRE(sensor.last_ranges[1] == Approx(2.8).margin(0.01));
}

TEST_CASE("Lidar scan rotates with sensor mount rotation", "[lidar]") {
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
    wall_tf.position = robcraft::engine::math::Vec3(3.0, 1.0, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(wall, wall_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.2, 1.0, 2.0)});

    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 3;
    sensor.range_max = 10.0;
    sensor.angle_min = -0.01;
    sensor.angle_max = 0.01;
    sensor.noise_stddev = 0.0;
    sensor.rotation = robcraft::engine::math::Vec3(0.0, 1.5707963, 0.0);
    sensor.rebuild_angles();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar::lidar_update(robot, sensor, world, rng);

    REQUIRE(sensor.last_ranges[1] == Approx(2.8).margin(0.01));
}

TEST_CASE("Lidar scan detects rotated colliders in the broad phase", "[lidar]") {
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
    wall_tf.position = robcraft::engine::math::Vec3(1.4, 1.0, 3.0);
    wall_tf.rotation = robcraft::engine::math::Quaternion::from_euler(0.0, 0.7853981634, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(wall, wall_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.1, 1.0, 2.0)});

    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 1;
    sensor.range_max = 10.0;
    sensor.angle_min = 0.0;
    sensor.angle_max = 0.0;
    sensor.rebuild_angles();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar::lidar_update(robot, sensor, world, rng);

    REQUIRE(sensor.last_ranges.size() == 1);
    REQUIRE(sensor.last_ranges[0] == Approx(1.515).margin(0.01));
}

TEST_CASE("Lidar noise adds variation", "[lidar]") {
    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 50;
    sensor.noise_stddev = 0.05;
    sensor.angle_min = -0.02;
    sensor.angle_max = 0.02;
    sensor.time_since_update = 10.0;
    sensor.rebuild_angles();

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
    robcraft::sensors::lidar::lidar_update(e, sensor, world, rng);

    // All 50 rays (FOV ±0.02 rad) hit the wall, so Gaussian noise must vary
    // the (finite) ranges; no ray may report an infinite no-detection range.
    bool has_variation = false;
    for (int i = 1; i < sensor.num_rays; ++i) {
        if (std::abs(sensor.last_ranges[i] - sensor.last_ranges[0]) > 0.001) {
            has_variation = true;
            break;
        }
    }
    REQUIRE(has_variation);
    for (int i = 0; i < sensor.num_rays; ++i) {
        REQUIRE(std::isfinite(sensor.last_ranges[i]));
    }
}

TEST_CASE("Lidar time accumulator", "[lidar]") {
    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.update_rate = 10.0;
    sensor.time_since_update = 0.0;

    sensor.time_since_update += 0.05;
    REQUIRE(sensor.time_since_update < 1.0 / sensor.update_rate);

    sensor.time_since_update += 0.06;
    REQUIRE(sensor.time_since_update >= 1.0 / sensor.update_rate);
}

TEST_CASE("Lidar keeps detecting past the origin-centered 25 m grid box", "[lidar]") {
    // Regression: the broad-phase grid was hardcoded to [-25, 25]; once the
    // robot crossed 25 m the ray origin fell outside the grid and every ray
    // reported +inf. The grid must follow the terrain footprint instead.
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    auto robot = world.create_entity();
    robcraft::engine::ecs::Transform3D robot_tf;
    robot_tf.position = robcraft::engine::math::Vec3(28.0, 0.25, 0.0);
    robot_tf.rotation =
        robcraft::engine::math::Quaternion::from_euler(0.0, 1.5707963, 0.0);  // face +X
    world.add_component<robcraft::engine::ecs::Transform3D>(robot, robot_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        robot,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.3, 0.15, 0.3)});

    auto wall = world.create_entity();
    robcraft::engine::ecs::Transform3D wall_tf;
    wall_tf.position = robcraft::engine::math::Vec3(31.0, 1.0, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(wall, wall_tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        wall,
        robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.2, 1.0, 2.0)});

    robcraft::sensors::lidar::LidarSensor2D sensor;
    sensor.num_rays = 3;
    sensor.range_max = 10.0;
    sensor.angle_min = -0.01;
    sensor.angle_max = 0.01;
    sensor.noise_stddev = 0.0;
    sensor.rebuild_angles();

    robcraft::engine::core::Random rng(42);
    robcraft::sensors::lidar::lidar_update(robot, sensor, world, rng);

    REQUIRE(sensor.last_ranges[1] == Approx(2.8).margin(0.01));
    REQUIRE(std::isfinite(sensor.last_ranges[0]));
    REQUIRE(std::isfinite(sensor.last_ranges[2]));
}
