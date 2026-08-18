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
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/ecs/vertical_motion.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/lighting/sky.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/engine/world/world_serializer.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::world;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::magnetometer;

using Catch::Approx;

TEST_CASE("WorldSerializer round-trip", "[serializer]") {
    const std::string path = "/tmp/test_world.world";

    SECTION("save and load preserves entities") {
        robcraft::engine::world::World world;
        auto e = world.create_entity();
        robcraft::engine::ecs::Transform3D tf;
        tf.position = robcraft::engine::math::Vec3(5.0, 1.0, -3.0);
        tf.scale = robcraft::engine::math::Vec3(2.0, 1.0, 0.5);
        world.add_component<robcraft::engine::ecs::Transform3D>(e, tf);
        world.add_component<robcraft::engine::ecs::Name>(e,
                                                         robcraft::engine::ecs::Name{"wall_test"});
        world.add_component<robcraft::engine::collision::BoxCollider>(
            e,
            robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(1.0, 0.5, 0.25)});

        REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

        robcraft::engine::world::World loaded;
        REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));

        size_t count = loaded.entities().max_allocated();
        bool found = false;
        for (size_t i = 1; i <= count; ++i) {
            robcraft::engine::core::Entity le = static_cast<robcraft::engine::core::Entity>(i);
            if (!loaded.valid(le)) continue;
            auto* n = loaded.get_component<robcraft::engine::ecs::Name>(le);
            if (n && n->value == "wall_test") {
                found = true;
                auto* t = loaded.get_component<robcraft::engine::ecs::Transform3D>(le);
                REQUIRE(t != nullptr);
                REQUIRE(t->position.x == 5.0);
                REQUIRE(t->position.z == -3.0);
            }
        }
        REQUIRE(found);
    }

    SECTION("load robot with lidar") {
        std::ofstream f(path);
        f << R"(world "test"
{
    robot "bot"
    {
        type = "differential_drive"
        position = [1, 0.5, 2]
        wheel_base = 0.42
        max_speed = 2.0
        lidar "scan"
        {
            range = 10
            fov = 270
            rays = 270
            rate = 10
        }
    }
})";
        f.close();

        robcraft::engine::world::World world;
        REQUIRE(robcraft::engine::world::WorldSerializer::load(world, path));

        size_t count = world.entities().max_allocated();
        bool found = false;
        for (size_t i = 1; i <= count; ++i) {
            robcraft::engine::core::Entity e = static_cast<robcraft::engine::core::Entity>(i);
            if (!world.valid(e)) continue;
            if (world.has_component<robcraft::robots::differential_drive::DifferentialDrive>(e)) {
                found = true;
                auto* dd =
                    world.get_component<robcraft::robots::differential_drive::DifferentialDrive>(e);
                REQUIRE(dd->wheel_base == 0.42);
                auto* lidar = world.get_component<robcraft::sensors::lidar::LidarSensor2D>(e);
                REQUIRE(lidar != nullptr);
                REQUIRE(lidar->range_max == 10.0);
                REQUIRE(lidar->num_rays == 270);
            }
        }
        REQUIRE(found);
    }

    SECTION("save and load sensor offsets") {
        robcraft::engine::world::World world;
        auto e = world.create_entity();
        world.add_component<robcraft::engine::ecs::Transform3D>(
            e, robcraft::engine::ecs::Transform3D{});
        world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"bot"});
        robcraft::robots::differential_drive::DifferentialDrive drive;
        world.add_component<robcraft::robots::differential_drive::DifferentialDrive>(e, drive);
        robcraft::sensors::lidar::LidarSensor2D lidar;
        lidar.position = robcraft::engine::math::Vec3(0.0, 0.15, 0.2);
        lidar.rotation = robcraft::engine::math::Vec3(0.0, 0.1, 0.0);
        world.add_component<robcraft::sensors::lidar::LidarSensor2D>(e, lidar);

        REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

        robcraft::engine::world::World loaded;
        REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));

        size_t count = loaded.entities().max_allocated();
        bool found = false;
        for (size_t i = 1; i <= count; ++i) {
            robcraft::engine::core::Entity le = static_cast<robcraft::engine::core::Entity>(i);
            if (!loaded.valid(le)) continue;
            auto* l = loaded.get_component<robcraft::sensors::lidar::LidarSensor2D>(le);
            if (l) {
                found = true;
                REQUIRE(l->position.x == 0.0);
                REQUIRE(l->position.y == Approx(0.15));
                REQUIRE(l->position.z == Approx(0.2));
                REQUIRE(l->rotation.y == Approx(0.1));
            }
        }
        REQUIRE(found);
    }

    SECTION("save and load a robot with a magnetometer") {
        robcraft::engine::world::World world;
        auto e = world.create_entity();
        world.add_component<robcraft::engine::ecs::Transform3D>(
            e, robcraft::engine::ecs::Transform3D{});
        world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"magbot"});
        world.add_component<robcraft::robots::differential_drive::DifferentialDrive>(
            e, robcraft::robots::differential_drive::DifferentialDrive{});
        robcraft::sensors::magnetometer::MagnetometerSensor mag;
        mag.update_rate = 20.0;
        mag.field_strength = 40.0;
        mag.declination_deg = 3.0;
        mag.inclination_deg = 55.0;
        mag.position = robcraft::engine::math::Vec3(0.1, 0.2, 0.3);
        mag.rotation = robcraft::engine::math::Vec3(0.0, 0.1, 0.0);
        world.add_component<robcraft::sensors::magnetometer::MagnetometerSensor>(e, mag);

        REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

        robcraft::engine::world::World loaded;
        REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));

        size_t count = loaded.entities().max_allocated();
        bool found = false;
        for (size_t i = 1; i <= count; ++i) {
            robcraft::engine::core::Entity le = static_cast<robcraft::engine::core::Entity>(i);
            if (!loaded.valid(le)) continue;
            auto* m = loaded.get_component<robcraft::sensors::magnetometer::MagnetometerSensor>(le);
            if (m) {
                found = true;
                REQUIRE(m->update_rate == Approx(20.0));
                REQUIRE(m->field_strength == Approx(40.0));
                REQUIRE(m->declination_deg == Approx(3.0));
                REQUIRE(m->inclination_deg == Approx(55.0));
                REQUIRE(m->position.x == Approx(0.1));
                REQUIRE(m->position.y == Approx(0.2));
                REQUIRE(m->position.z == Approx(0.3));
                REQUIRE(m->rotation.y == Approx(0.1));
            }
        }
        REQUIRE(found);
    }

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer resolves relative terrain path against world dir", "[serializer]") {
    const std::string dir = "/tmp/robcraft_worldtest";
    std::filesystem::create_directories(dir);
    const std::string tpath = dir + "/level.terrain";
    robcraft::engine::world::Terrain t(4, 4, 1.0);
    t.set_height(1, 1, 2.0f);
    REQUIRE(save_terrain(t, tpath));

    const std::string wpath = dir + "/level.world";
    {
        std::ofstream f(wpath);
        f << "world \"t\"\n{\n  terrain = \"level.terrain\"\n}\n";
    }

    robcraft::engine::world::World world;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(world, wpath));
    REQUIRE(world.has_terrain());
    REQUIRE(world.terrain().height_at(1, 1) == Approx(2.0f));

    std::filesystem::remove_all(dir);
}

TEST_CASE("WorldSerializer saves terrain path relative to the world file", "[serializer]") {
    const std::string dir = "/tmp/robcraft_worldtest";
    std::filesystem::create_directories(dir);
    const std::string wpath = dir + "/level.world";

    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(4, 4, 1.0));
    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, wpath));

    REQUIRE(std::filesystem::exists(dir + "/level.world.terrain"));
    std::ifstream f(wpath);
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    REQUIRE(content.find("terrain = \"level.world.terrain\"") != std::string::npos);
    REQUIRE(content.find(dir) == std::string::npos);

    std::filesystem::remove_all(dir);
}

TEST_CASE("WorldSerializer resolves relative terrain path against world dir, not CWD",
          "[serializer]") {
    const std::string dir = "/tmp/robcraft_worldtest";
    std::filesystem::create_directories(dir);
    const std::string tpath = dir + "/level.terrain";
    robcraft::engine::world::Terrain t(4, 4, 1.0);
    t.set_height(1, 1, 2.0f);
    REQUIRE(save_terrain(t, tpath));

    // A decoy terrain in the CWD must not shadow the world-dir-relative file.
    const std::string decoy = (std::filesystem::current_path() / "level.terrain").string();
    {
        robcraft::engine::world::Terrain decoy_t(4, 4, 1.0);
        REQUIRE(save_terrain(decoy_t, decoy));
    }

    const std::string wpath = dir + "/level.world";
    {
        std::ofstream f(wpath);
        f << "world \"t\"\n{\n  terrain = \"level.terrain\"\n}\n";
    }

    robcraft::engine::world::World world;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(world, wpath));
    REQUIRE(world.has_terrain());
    REQUIRE(world.terrain().height_at(1, 1) == Approx(2.0f));

    std::filesystem::remove(decoy);
    std::filesystem::remove_all(dir);
}

TEST_CASE("WorldSerializer keeps absolute terrain path", "[serializer]") {
    const std::string dir = "/tmp/robcraft_worldtest";
    std::filesystem::create_directories(dir);
    const std::string tpath = dir + "/level.terrain";
    robcraft::engine::world::Terrain t(4, 4, 1.0);
    t.set_height(1, 1, 2.0f);
    REQUIRE(save_terrain(t, tpath));

    const std::string wpath = dir + "/level.world";
    {
        std::ofstream f(wpath);
        f << "world \"t\"\n{\n  terrain = \"" << tpath << "\"\n}\n";
    }

    robcraft::engine::world::World world;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(world, wpath));
    REQUIRE(world.has_terrain());
    REQUIRE(world.terrain().height_at(1, 1) == Approx(2.0f));

    std::filesystem::remove_all(dir);
}

TEST_CASE("WorldSerializer writes mesh labels for nature prefixes", "[serializer]") {
    const std::string path = "/tmp/test_world_nature.world";

    auto save_entity = [&](const std::string& name) {
        robcraft::engine::world::World world;
        auto e = world.create_entity();
        robcraft::engine::ecs::Transform3D tf;
        tf.position = robcraft::engine::math::Vec3(1.0, 0.0, 2.0);
        world.add_component<robcraft::engine::ecs::Transform3D>(e, tf);
        world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{name});
        world.add_component<robcraft::engine::collision::BoxCollider>(
            e,
            robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(0.5, 0.5, 0.5)});
        REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));
        std::ifstream f(path);
        std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        return content;
    };

    REQUIRE(save_entity("tree_2_7").find("mesh = \"tree_2\"") != std::string::npos);
    REQUIRE(save_entity("tree_4_1").find("mesh = \"tree_4\"") != std::string::npos);
    REQUIRE(save_entity("pine_1_5").find("mesh = \"pine\"") != std::string::npos);
    REQUIRE(save_entity("twisted_3_2").find("mesh = \"twisted\"") != std::string::npos);
    REQUIRE(save_entity("dead_5_9").find("mesh = \"dead\"") != std::string::npos);
    REQUIRE(save_entity("bush_2_3").find("mesh = \"bush_2\"") != std::string::npos);
    REQUIRE(save_entity("couch_1_2").find("mesh = \"couch\"") != std::string::npos);
    REQUIRE(save_entity("light_floor_1").find("mesh = \"light_floor\"") != std::string::npos);
    REQUIRE(save_entity("shelf_2_1").find("mesh = \"shelf\"") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer round-trips lighting and point lights", "[serializer]") {
    const std::string path = "/tmp/test_world_lights.world";

    robcraft::engine::world::World world;
    robcraft::engine::lighting::SceneLighting l;
    l.sun_direction = robcraft::engine::math::Vec3(0.2, 1.0, 0.1);
    l.sun_intensity = 2.5f;
    l.shadows_enabled = false;
    world.set_lighting(l);

    auto e = world.create_entity();
    robcraft::engine::ecs::Transform3D tf;
    tf.position = robcraft::engine::math::Vec3(3.0, 1.5, 4.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(e, tf);
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"lamp"});
    robcraft::engine::lighting::PointLight pl;
    pl.color = robcraft::engine::math::Vec3(1.0f, 0.5f, 0.25f);
    pl.intensity = 2.0f;
    pl.range = 12.0f;
    world.add_component<robcraft::engine::lighting::PointLight>(e, pl);

    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

    robcraft::engine::world::World loaded;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));

    REQUIRE(loaded.lighting().sun_direction.x == Approx(0.2));
    REQUIRE(loaded.lighting().sun_intensity == Approx(2.5f));
    REQUIRE(!loaded.lighting().shadows_enabled);

    size_t count = loaded.entities().max_allocated();
    bool found = false;
    for (size_t i = 1; i <= count; ++i) {
        robcraft::engine::core::Entity le = static_cast<robcraft::engine::core::Entity>(i);
        if (!loaded.valid(le)) continue;
        auto* p = loaded.get_component<robcraft::engine::lighting::PointLight>(le);
        if (p) {
            found = true;
            REQUIRE(p->color.x == Approx(1.0f));
            REQUIRE(p->intensity == Approx(2.0f));
            REQUIRE(p->range == Approx(12.0f));
        }
    }
    REQUIRE(found);

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer round-trips depth camera and 3D lidar", "[serializer]") {
    const std::string path = "/tmp/test_world_sensors.world";

    robcraft::engine::world::World world;
    auto e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"bot"});
    world.add_component<robcraft::robots::differential_drive::DifferentialDrive>(
        e, robcraft::robots::differential_drive::DifferentialDrive{});
    robcraft::sensors::depth_camera::DepthCameraSensor dc;
    dc.width = 320;
    dc.height = 240;
    dc.rebuild();
    world.add_component<robcraft::sensors::depth_camera::DepthCameraSensor>(e, dc);
    robcraft::sensors::lidar3d::LidarSensor3D l3;
    l3.horizontal_rays = 180;
    l3.vertical_beams = 8;
    l3.rebuild();
    world.add_component<robcraft::sensors::lidar3d::LidarSensor3D>(e, l3);

    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

    robcraft::engine::world::World loaded;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));

    size_t count = loaded.entities().max_allocated();
    bool found = false;
    for (size_t i = 1; i <= count; ++i) {
        robcraft::engine::core::Entity le = static_cast<robcraft::engine::core::Entity>(i);
        if (!loaded.valid(le)) continue;
        auto* d = loaded.get_component<robcraft::sensors::depth_camera::DepthCameraSensor>(le);
        auto* l = loaded.get_component<robcraft::sensors::lidar3d::LidarSensor3D>(le);
        if (d && l) {
            found = true;
            REQUIRE(d->width == 320);
            REQUIRE(d->height == 240);
            REQUIRE(d->depth_data.size() == static_cast<size_t>(320) * 240);
            REQUIRE(l->horizontal_rays == 180);
            REQUIRE(l->vertical_beams == 8);
            REQUIRE(l->last_ranges.size() == static_cast<size_t>(180) * 8);
        }
    }
    REQUIRE(found);

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer round-trips the sky block", "[serializer]") {
    const std::string path = "/tmp/test_world_sky.world";

    robcraft::engine::world::World world;
    robcraft::engine::lighting::Sky s;
    s.zenith_color = robcraft::engine::math::Vec3(0.12f, 0.25f, 0.55f);
    s.horizon_color = robcraft::engine::math::Vec3(0.9f, 0.6f, 0.3f);
    world.set_sky(s);
    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

    robcraft::engine::world::World loaded;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));
    REQUIRE(loaded.sky().zenith_color.x == Approx(0.12f));
    REQUIRE(loaded.sky().zenith_color.y == Approx(0.25f));
    REQUIRE(loaded.sky().zenith_color.z == Approx(0.55f));
    REQUIRE(loaded.sky().horizon_color.x == Approx(0.9f));
    REQUIRE(loaded.sky().horizon_color.y == Approx(0.6f));
    REQUIRE(loaded.sky().horizon_color.z == Approx(0.3f));

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer keeps default sky without a sky block", "[serializer]") {
    const std::string path = "/tmp/test_world_sky_default.world";

    robcraft::engine::world::World world;
    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

    robcraft::engine::world::World loaded;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));
    REQUIRE(loaded.sky().horizon_color.z == Approx(0.9f));
    REQUIRE(loaded.sky().zenith_color.x == Approx(0.4f));

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer round-trips the physics block", "[serializer]") {
    const std::string path = "/tmp/test_world_physics.world";

    robcraft::engine::world::World world;
    world.set_gravity(3.7);
    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

    robcraft::engine::world::World loaded;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));
    REQUIRE(loaded.gravity() == Approx(3.7));

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer keeps default gravity without a physics block", "[serializer]") {
    const std::string path = "/tmp/test_world_physics_default.world";

    robcraft::engine::world::World world;
    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

    robcraft::engine::world::World loaded;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));
    REQUIRE(loaded.gravity() == Approx(9.81));

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer gives loaded robots a VerticalMotion component", "[serializer]") {
    const std::string path = "/tmp/test_world_vm.world";

    std::ofstream f(path);
    f << "world \"test\"\n{\n    robot \"bot\"\n    {\n"
         "        type = \"differential_drive\"\n        position = [1, 0.5, 2]\n"
         "    }\n}\n";
    f.close();

    robcraft::engine::world::World world;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(world, path));

    size_t count = world.entities().max_allocated();
    bool found = false;
    for (size_t i = 1; i <= count; ++i) {
        robcraft::engine::core::Entity e = static_cast<robcraft::engine::core::Entity>(i);
        if (!world.valid(e)) continue;
        if (world.has_component<robcraft::robots::differential_drive::DifferentialDrive>(e)) {
            found = true;
            auto* vm = world.get_component<robcraft::engine::ecs::VerticalMotion>(e);
            REQUIRE(vm != nullptr);
            REQUIRE(vm->vertical_velocity == Approx(0.0));
        }
    }
    REQUIRE(found);

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer round-trips sensor noise values", "[serializer]") {
    const std::string path = "/tmp/test_world_noise.world";

    robcraft::engine::world::World world;
    auto e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"bot"});
    robcraft::robots::differential_drive::DifferentialDrive drive;
    drive.odom_noise_stddev = 0.4;
    world.add_component<robcraft::robots::differential_drive::DifferentialDrive>(e, drive);

    robcraft::sensors::lidar::LidarSensor2D lidar;
    lidar.noise_stddev = 0.3;
    lidar.rebuild_angles();
    world.add_component<robcraft::sensors::lidar::LidarSensor2D>(e, lidar);

    robcraft::sensors::lidar3d::LidarSensor3D l3;
    l3.noise_stddev = 0.3;
    l3.rebuild();
    world.add_component<robcraft::sensors::lidar3d::LidarSensor3D>(e, l3);

    robcraft::sensors::imu::ImuSensor imu;
    imu.linear_acceleration_noise_stddev = 0.1;
    imu.angular_velocity_noise_stddev = 0.2;
    world.add_component<robcraft::sensors::imu::ImuSensor>(e, imu);

    REQUIRE(robcraft::engine::world::WorldSerializer::save(world, path));

    robcraft::engine::world::World loaded;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(loaded, path));

    size_t count = loaded.entities().max_allocated();
    bool found = false;
    for (size_t i = 1; i <= count; ++i) {
        robcraft::engine::core::Entity le = static_cast<robcraft::engine::core::Entity>(i);
        if (!loaded.valid(le)) continue;
        auto* dd =
            loaded.get_component<robcraft::robots::differential_drive::DifferentialDrive>(le);
        auto* li = loaded.get_component<robcraft::sensors::lidar::LidarSensor2D>(le);
        auto* l3b = loaded.get_component<robcraft::sensors::lidar3d::LidarSensor3D>(le);
        auto* im = loaded.get_component<robcraft::sensors::imu::ImuSensor>(le);
        if (dd && li && l3b && im) {
            found = true;
            REQUIRE(dd->odom_noise_stddev == Approx(0.4));
            REQUIRE(li->noise_stddev == Approx(0.3));
            REQUIRE(l3b->noise_stddev == Approx(0.3));
            REQUIRE(im->linear_acceleration_noise_stddev == Approx(0.1));
            REQUIRE(im->angular_velocity_noise_stddev == Approx(0.2));
        }
    }
    REQUIRE(found);

    std::remove(path.c_str());
}

TEST_CASE("WorldSerializer legacy imu noise key sets both noise fields", "[serializer]") {
    const std::string path = "/tmp/test_world_imu_noise.world";

    std::ofstream f(path);
    f << "world \"test\"\n{\n    robot \"bot\"\n    {\n"
         "        type = \"differential_drive\"\n        position = [0, 0, 0]\n"
         "        imu \"imu\"\n        {\n            rate = 100\n            noise = 0.05\n"
         "        }\n    }\n}\n";
    f.close();

    robcraft::engine::world::World world;
    REQUIRE(robcraft::engine::world::WorldSerializer::load(world, path));

    size_t count = world.entities().max_allocated();
    bool found = false;
    for (size_t i = 1; i <= count; ++i) {
        robcraft::engine::core::Entity e = static_cast<robcraft::engine::core::Entity>(i);
        if (!world.valid(e)) continue;
        auto* im = world.get_component<robcraft::sensors::imu::ImuSensor>(e);
        if (im) {
            found = true;
            REQUIRE(im->linear_acceleration_noise_stddev == Approx(0.05));
            REQUIRE(im->angular_velocity_noise_stddev == Approx(0.05));
        }
    }
    REQUIRE(found);

    std::remove(path.c_str());
}
