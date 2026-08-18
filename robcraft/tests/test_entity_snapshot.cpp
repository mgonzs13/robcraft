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
#include <memory>

#include "robcraft/editor/command/entity_snapshot.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

using namespace robcraft::editor::command;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::world;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::lidar3d;

using Catch::Approx;

TEST_CASE("EntitySnapshot captures and restores all components", "[undo]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity e = world.create_entity();

    world.add_component<robcraft::engine::ecs::Name>(e,
                                                     robcraft::engine::ecs::Name{"robot_mike_7"});
    robcraft::engine::ecs::Transform3D tf;
    tf.position = robcraft::engine::math::Vec3(1.0, 2.0, 3.0);
    tf.rotation = robcraft::engine::math::Quaternion::from_euler(0.0, 0.5, 0.0);
    tf.scale = robcraft::engine::math::Vec3(2.0, 2.0, 2.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(e, tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        e, robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.5, 0.25, 0.5)});
    world.add_component<robcraft::robots::differential_drive::DifferentialDrive>(
        e, robcraft::robots::differential_drive::DifferentialDrive{});
    world.add_component<robcraft::sensors::lidar::LidarSensor2D>(
        e, robcraft::sensors::lidar::LidarSensor2D{});
    world.add_component<robcraft::sensors::imu::ImuSensor>(e, robcraft::sensors::imu::ImuSensor{});
    world.add_component<robcraft::sensors::gps::GpsSensor>(e, robcraft::sensors::gps::GpsSensor{});
    world.add_component<robcraft::sensors::magnetometer::MagnetometerSensor>(
        e, robcraft::sensors::magnetometer::MagnetometerSensor{});
    world.add_component<robcraft::sensors::camera::CameraSensor>(
        e, robcraft::sensors::camera::CameraSensor{});
    world.add_component<robcraft::sensors::depth_camera::DepthCameraSensor>(
        e, robcraft::sensors::depth_camera::DepthCameraSensor{});
    world.add_component<robcraft::sensors::lidar3d::LidarSensor3D>(
        e, robcraft::sensors::lidar3d::LidarSensor3D{});
    world.add_component<robcraft::engine::lighting::PointLight>(
        e, robcraft::engine::lighting::PointLight{});

    auto snap = robcraft::editor::command::EntitySnapshot::capture(world, e);
    REQUIRE(snap != nullptr);
    REQUIRE(snap->entity() == e);

    // Mutate the entity, then restore the snapshot.
    world.get_component<robcraft::engine::ecs::Transform3D>(e)->position =
        robcraft::engine::math::Vec3(9.0, 9.0, 9.0);
    world.remove_component<robcraft::sensors::lidar::LidarSensor2D>(e);

    snap->restore(world);

    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "robot_mike_7");
    auto* restored_tf = world.get_component<robcraft::engine::ecs::Transform3D>(e);
    REQUIRE(restored_tf->position.x == Approx(1.0));
    REQUIRE(restored_tf->position.y == Approx(2.0));
    REQUIRE(restored_tf->position.z == Approx(3.0));
    REQUIRE(restored_tf->rotation.to_euler().y == Approx(0.5));
    REQUIRE(restored_tf->scale.x == Approx(2.0));
    REQUIRE(world.has_component<robcraft::engine::collision::BoxCollider>(e));
    REQUIRE(world.has_component<robcraft::robots::differential_drive::DifferentialDrive>(e));
    REQUIRE(world.has_component<robcraft::sensors::lidar::LidarSensor2D>(e));
    REQUIRE(world.has_component<robcraft::sensors::imu::ImuSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::gps::GpsSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::magnetometer::MagnetometerSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::camera::CameraSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::depth_camera::DepthCameraSensor>(e));
    REQUIRE(world.has_component<robcraft::sensors::lidar3d::LidarSensor3D>(e));
    REQUIRE(world.has_component<robcraft::engine::lighting::PointLight>(e));
}

TEST_CASE("EntitySnapshot restore recreates a destroyed entity", "[undo]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"wall_3"});
    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});

    auto snap = robcraft::editor::command::EntitySnapshot::capture(world, e);
    world.destroy_entity(e);
    REQUIRE(!world.valid(e));

    snap->restore(world);
    REQUIRE(world.valid(e));
    REQUIRE(world.valid(snap->entity()));
    REQUIRE(snap->entity() == e);  // EntityManager reuses the freed id
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "wall_3");
}

TEST_CASE("EntitySnapshot restore reclaims its recorded id", "[undo]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"crate_1"});
    auto snap = robcraft::editor::command::EntitySnapshot::capture(world, e);

    // Free a blocker id BEFORE e so the free-list front is not e's id when
    // restore creates a replacement entity. Restore must still claim e's id
    // rather than stealing the free-list front.
    robcraft::engine::core::Entity blocker = world.create_entity();
    world.destroy_entity(blocker);
    world.destroy_entity(e);

    snap->restore(world);
    REQUIRE(world.valid(snap->entity()));
    REQUIRE(snap->entity() == e);  // reclaimed the recorded id
    REQUIRE(!world.valid(blocker));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(snap->entity())->value == "crate_1");
}

TEST_CASE("EntitySnapshot restore prunes components added after capture", "[undo]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"pruned_1"});
    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});

    auto snap = robcraft::editor::command::EntitySnapshot::capture(world, e);

    world.add_component<robcraft::engine::collision::BoxCollider>(
        e, robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.5, 0.25, 0.5)});
    world.add_component<robcraft::robots::differential_drive::DifferentialDrive>(
        e, robcraft::robots::differential_drive::DifferentialDrive{});

    snap->restore(world);

    REQUIRE(!world.has_component<robcraft::engine::collision::BoxCollider>(e));
    REQUIRE(!world.has_component<robcraft::robots::differential_drive::DifferentialDrive>(e));
    REQUIRE(world.has_component<robcraft::engine::ecs::Name>(e));
    REQUIRE(world.has_component<robcraft::engine::ecs::Transform3D>(e));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "pruned_1");
}

TEST_CASE("EntitySnapshot restore of an empty snapshot strips all components", "[undo]") {
    robcraft::engine::world::World world;
    robcraft::engine::core::Entity e = world.create_entity();

    auto snap = robcraft::editor::command::EntitySnapshot::capture(world, e);
    REQUIRE(snap != nullptr);
    REQUIRE(snap->entity() == e);

    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"wipe_me"});
    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::collision::BoxCollider>(
        e, robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.5, 0.25, 0.5)});
    snap->restore(world);

    REQUIRE(world.valid(e));
    REQUIRE(!world.has_component<robcraft::engine::ecs::Name>(e));
    REQUIRE(!world.has_component<robcraft::engine::ecs::Transform3D>(e));
    REQUIRE(!world.has_component<robcraft::engine::collision::BoxCollider>(e));
}

TEST_CASE("EntitySnapshot capture of invalid entity is null", "[undo]") {
    robcraft::engine::world::World world;
    auto snap = robcraft::editor::command::EntitySnapshot::capture(
        world, robcraft::engine::core::INVALID_ENTITY);
    REQUIRE(snap == nullptr);
}
