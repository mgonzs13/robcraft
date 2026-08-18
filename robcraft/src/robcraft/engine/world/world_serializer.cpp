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

#include "robcraft/engine/world/world_serializer.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/ecs/vertical_motion.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/lighting/sky.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/world/terrain_io.hpp"
#include "robcraft/engine/world/world_tokenizer.hpp"
#include "robcraft/renderer/model_paths.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

namespace robcraft::engine::world {

using namespace robcraft::engine::world;
using namespace robcraft::engine::collision;
using namespace robcraft::engine::core;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;

static Vec3 parse_vec3(Tokenizer& t) {
    t.expect(TokenType::LBracket);
    double x = std::stod(t.next().text);
    t.expect(TokenType::Comma);
    double y = std::stod(t.next().text);
    t.expect(TokenType::Comma);
    double z = std::stod(t.next().text);
    t.expect(TokenType::RBracket);
    return Vec3(x, y, z);
}

static void parse_lidar(Tokenizer& t, LidarSensor2D& lidar) {
    t.expect(TokenType::LBrace);
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "offset")
            lidar.position = parse_vec3(t);
        else if (key == "rotation")
            lidar.rotation = parse_vec3(t);
        else {
            double val = std::stod(t.next().text);
            if (key == "range")
                lidar.range_max = val;
            else if (key == "range_min")
                lidar.range_min = val;
            else if (key == "fov") {
                lidar.angle_max = robcraft::engine::math::deg_to_rad(val * 0.5);
                lidar.angle_min = -lidar.angle_max;
            } else if (key == "rays")
                lidar.num_rays = static_cast<int>(val);
            else if (key == "rate")
                lidar.update_rate = val;
            else if (key == "noise")
                lidar.noise_stddev = val;
        }
    }
    t.expect(TokenType::RBrace);
    lidar.rebuild_angles();
}

/**
 * @brief Parses a shared camera-style sensor block (camera / depth camera).
 * @tparam T Sensor type exposing position, rotation, width, height, fov_deg,
 * update_rate, near_plane, far_plane, and rebuild().
 * @param t Tokenizer positioned at the opening brace.
 * @param cam Sensor to populate.
 */
template <typename T>
static void parse_camera_like(Tokenizer& t, T& cam) {
    t.expect(TokenType::LBrace);
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "offset")
            cam.position = parse_vec3(t);
        else if (key == "rotation")
            cam.rotation = parse_vec3(t);
        else {
            double val = std::stod(t.next().text);
            if (key == "width")
                cam.width = static_cast<int>(val);
            else if (key == "height")
                cam.height = static_cast<int>(val);
            else if (key == "fov")
                cam.fov_deg = val;
            else if (key == "rate")
                cam.update_rate = val;
            else if (key == "near")
                cam.near_plane = val;
            else if (key == "far")
                cam.far_plane = val;
        }
    }
    t.expect(TokenType::RBrace);
    cam.rebuild();
}

static void parse_camera(Tokenizer& t, CameraSensor& cam) {
    parse_camera_like(t, cam);
}

static void parse_depth_camera(Tokenizer& t, DepthCameraSensor& cam) {
    parse_camera_like(t, cam);
}

static void parse_lidar3d(Tokenizer& t, LidarSensor3D& lidar) {
    t.expect(TokenType::LBrace);
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "offset")
            lidar.position = parse_vec3(t);
        else if (key == "rotation")
            lidar.rotation = parse_vec3(t);
        else {
            double val = std::stod(t.next().text);
            if (key == "range")
                lidar.range_max = val;
            else if (key == "hfov") {
                lidar.horizontal_fov_max = robcraft::engine::math::deg_to_rad(val * 0.5);
                lidar.horizontal_fov_min = -lidar.horizontal_fov_max;
            } else if (key == "vfov") {
                lidar.vertical_fov_max = robcraft::engine::math::deg_to_rad(val * 0.5);
                lidar.vertical_fov_min = -lidar.vertical_fov_max;
            } else if (key == "rays")
                lidar.horizontal_rays = static_cast<int>(val);
            else if (key == "beams")
                lidar.vertical_beams = static_cast<int>(val);
            else if (key == "rate")
                lidar.update_rate = val;
            else if (key == "noise")
                lidar.noise_stddev = val;
        }
    }
    t.expect(TokenType::RBrace);
    lidar.rebuild();
}

static void parse_imu(Tokenizer& t, ImuSensor& imu) {
    t.expect(TokenType::LBrace);
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "offset")
            imu.position = parse_vec3(t);
        else if (key == "rotation")
            imu.rotation = parse_vec3(t);
        else {
            double val = std::stod(t.next().text);
            if (key == "rate")
                imu.update_rate = val;
            else if (key == "noise") {
                imu.linear_acceleration_noise_stddev = val;
                imu.angular_velocity_noise_stddev = val;
            } else if (key == "linear_noise")
                imu.linear_acceleration_noise_stddev = val;
            else if (key == "angular_noise")
                imu.angular_velocity_noise_stddev = val;
        }
    }
    t.expect(TokenType::RBrace);
}

static void parse_gps(Tokenizer& t, GpsSensor& gps) {
    t.expect(TokenType::LBrace);
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "offset")
            gps.position = parse_vec3(t);
        else if (key == "rotation")
            gps.rotation = parse_vec3(t);
        else {
            double val = std::stod(t.next().text);
            if (key == "rate")
                gps.update_rate = val;
            else if (key == "noise")
                gps.position_noise_stddev = val;
        }
    }
    t.expect(TokenType::RBrace);
}

/**
 * @brief Parses a magnetometer sensor block inside a robot.
 * @param t Tokenizer positioned at the opening brace.
 * @param mag Sensor to populate.
 */
static void parse_magnetometer(Tokenizer& t, MagnetometerSensor& mag) {
    t.expect(TokenType::LBrace);
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "offset")
            mag.position = parse_vec3(t);
        else if (key == "rotation")
            mag.rotation = parse_vec3(t);
        else {
            double val = std::stod(t.next().text);
            if (key == "rate")
                mag.update_rate = val;
            else if (key == "noise")
                mag.magnetic_field_noise_stddev = val;
            else if (key == "field")
                mag.field_strength = val;
            else if (key == "declination")
                mag.declination_deg = val;
            else if (key == "inclination")
                mag.inclination_deg = val;
        }
    }
    t.expect(TokenType::RBrace);
}

static void parse_robot(Tokenizer& t, World& world) {
    std::string name;
    if (t.peek().type == TokenType::String) {
        name = t.next().text;
    }
    t.expect(TokenType::LBrace);

    auto e = world.create_entity();
    world.add_component<Name>(e, Name{name});

    Transform3D tf;
    DifferentialDrive drive;
    bool has_drive = false;

    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        if (key == "type") {
            t.expect(TokenType::Equals);
            t.expect(TokenType::String);
            has_drive = true;
        } else if (key == "position") {
            t.expect(TokenType::Equals);
            tf.position = parse_vec3(t);
        } else if (key == "rotation") {
            t.expect(TokenType::Equals);
            Vec3 euler = parse_vec3(t);
            tf.rotation = Quaternion::from_euler(euler.x, euler.y, euler.z);
        } else if (key == "scale") {
            t.expect(TokenType::Equals);
            tf.scale = parse_vec3(t);
        } else if (key == "wheel_base") {
            t.expect(TokenType::Equals);
            drive.wheel_base = std::stod(t.next().text);
        } else if (key == "max_speed") {
            t.expect(TokenType::Equals);
            drive.max_linear_speed = std::stod(t.next().text);
        } else if (key == "odom_rate") {
            t.expect(TokenType::Equals);
            drive.odom_rate = std::stod(t.next().text);
        } else if (key == "odom_noise") {
            t.expect(TokenType::Equals);
            drive.odom_noise_stddev = std::stod(t.next().text);
        } else if (key == "lidar") {
            t.expect(TokenType::String);
            LidarSensor2D lidar;
            parse_lidar(t, lidar);
            world.add_component<LidarSensor2D>(e, lidar);
        } else if (key == "camera") {
            t.expect(TokenType::String);
            CameraSensor cam;
            parse_camera(t, cam);
            world.add_component<CameraSensor>(e, cam);
        } else if (key == "imu") {
            t.expect(TokenType::String);
            ImuSensor imu;
            parse_imu(t, imu);
            world.add_component<ImuSensor>(e, imu);
        } else if (key == "gps") {
            t.expect(TokenType::String);
            GpsSensor gps;
            parse_gps(t, gps);
            world.add_component<GpsSensor>(e, gps);
        } else if (key == "magnetometer") {
            t.expect(TokenType::String);
            MagnetometerSensor mag;
            parse_magnetometer(t, mag);
            world.add_component<MagnetometerSensor>(e, mag);
        } else if (key == "depth_camera") {
            t.expect(TokenType::String);
            DepthCameraSensor cam;
            parse_depth_camera(t, cam);
            world.add_component<DepthCameraSensor>(e, cam);
        } else if (key == "lidar3d") {
            t.expect(TokenType::String);
            LidarSensor3D lidar;
            parse_lidar3d(t, lidar);
            world.add_component<LidarSensor3D>(e, lidar);
        }
    }
    t.expect(TokenType::RBrace);

    if (tf.scale.x == 0.0) tf.scale = Vec3(0.6, 0.3, 0.6);
    world.add_component<Transform3D>(e, tf);
    if (has_drive) {
        world.add_component<DifferentialDrive>(e, drive);
        world.add_component<VerticalMotion>(e, VerticalMotion{});
    }
    world.add_component<BoxCollider>(e, BoxCollider{Vec3(0.3, 0.15, 0.3)});
}

static void parse_object(Tokenizer& t, World& world) {
    std::string name;
    if (t.peek().type == TokenType::String) {
        name = t.next().text;
    }
    t.expect(TokenType::LBrace);

    auto e = world.create_entity();
    world.add_component<Name>(e, Name{name});

    Transform3D tf;
    bool has_collider = false;
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        if (key == "mesh") {
            t.expect(TokenType::Equals);
            t.expect(TokenType::String);
        } else if (key == "position") {
            t.expect(TokenType::Equals);
            tf.position = parse_vec3(t);
        } else if (key == "scale") {
            t.expect(TokenType::Equals);
            tf.scale = parse_vec3(t);
        } else if (key == "collider") {
            has_collider = true;
            t.expect(TokenType::Equals);
            t.expect(TokenType::String);
        } else if (key == "rotation") {
            t.expect(TokenType::Equals);
            Vec3 euler = parse_vec3(t);
            tf.rotation = Quaternion::from_euler(euler.x, euler.y, euler.z);
        }
    }
    t.expect(TokenType::RBrace);

    world.add_component<Transform3D>(e, tf);
    if (has_collider) {
        world.add_component<BoxCollider>(e, BoxCollider{tf.scale * 0.5});
    }
}

static void parse_lighting(Tokenizer& t, World& world) {
    t.expect(TokenType::LBrace);
    SceneLighting l = world.lighting();
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "sun_direction")
            l.sun_direction = parse_vec3(t);
        else if (key == "sun_color")
            l.sun_color = parse_vec3(t);
        else if (key == "ambient_color")
            l.ambient_color = parse_vec3(t);
        else if (key == "shadows") {
            std::string val = t.next().text;
            l.shadows_enabled = val == "true" || val == "1";
        } else {
            double val = std::stod(t.next().text);
            if (key == "sun_intensity")
                l.sun_intensity = static_cast<float>(val);
            else if (key == "ambient_intensity")
                l.ambient_intensity = static_cast<float>(val);
        }
    }
    t.expect(TokenType::RBrace);
    world.set_lighting(l);
}

static void parse_sky(Tokenizer& t, World& world) {
    t.expect(TokenType::LBrace);
    Sky sky = world.sky();
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "zenith_color")
            sky.zenith_color = parse_vec3(t);
        else if (key == "horizon_color")
            sky.horizon_color = parse_vec3(t);
    }
    t.expect(TokenType::RBrace);
    world.set_sky(sky);
}

static void parse_physics(Tokenizer& t, World& world) {
    t.expect(TokenType::LBrace);
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        double val = std::stod(t.next().text);
        if (key == "gravity") world.set_gravity(val);
    }
    t.expect(TokenType::RBrace);
}

static void parse_light(Tokenizer& t, World& world) {
    std::string name;
    if (t.peek().type == TokenType::String) {
        name = t.next().text;
    }
    t.expect(TokenType::LBrace);

    auto e = world.create_entity();
    world.add_component<Name>(e, Name{name});

    Transform3D tf;
    PointLight light;
    while (t.peek().type != TokenType::RBrace) {
        std::string key = t.next().text;
        t.expect(TokenType::Equals);
        if (key == "position")
            tf.position = parse_vec3(t);
        else if (key == "rotation") {
            Vec3 euler = parse_vec3(t);
            tf.rotation = Quaternion::from_euler(euler.x, euler.y, euler.z);
        } else if (key == "color")
            light.color = parse_vec3(t);
        else {
            double val = std::stod(t.next().text);
            if (key == "intensity")
                light.intensity = static_cast<float>(val);
            else if (key == "range")
                light.range = static_cast<float>(val);
        }
    }
    t.expect(TokenType::RBrace);

    world.add_component<Transform3D>(e, tf);
    world.add_component<PointLight>(e, light);
}

bool WorldSerializer::load(World& world, const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        auto log = get_logger();
        log->error("Cannot open world file: {}", path);
        return false;
    }

    try {
        Tokenizer t(file);
        if (t.peek().type == TokenType::Ident && t.peek().text == "world") {
            t.next();
            if (t.peek().type == TokenType::String) t.next();
            t.expect(TokenType::LBrace);
        }

        while (t.peek().type != TokenType::RBrace && t.peek().type != TokenType::End) {
            std::string block = t.next().text;
            if (block == "robot")
                parse_robot(t, world);
            else if (block == "object")
                parse_object(t, world);
            else if (block == "light")
                parse_light(t, world);
            else if (block == "lighting")
                parse_lighting(t, world);
            else if (block == "sky")
                parse_sky(t, world);
            else if (block == "physics")
                parse_physics(t, world);
            else if (block == "terrain") {
                t.expect(TokenType::Equals);
                std::string terrain_path = t.next().text;
                std::filesystem::path tp(terrain_path);
                std::filesystem::path resolved = tp;
                if (tp.is_relative()) {
                    std::filesystem::path wdir = std::filesystem::path(path).parent_path();
                    if (!wdir.empty()) resolved = wdir / tp;
                }
                Terrain terrain;
                load_terrain(terrain, resolved.string());
                world.set_terrain(std::move(terrain));
            }
        }
        return true;
    } catch (const std::exception& e) {
        auto log = get_logger();
        log->error("Parse error: {}", e.what());
        return false;
    }
}

static void write_vec3(std::ostream& out, const Vec3& v) {
    out << "[" << v.x << ", " << v.y << ", " << v.z << "]";
}

/**
 * @brief Writes a shared camera-style sensor block (camera / depth camera).
 * @tparam T Sensor type exposing position, rotation, width, height, fov_deg,
 * update_rate, near_plane, and far_plane.
 * @param file Output stream.
 * @param kind Block keyword, e.g. "camera".
 * @param name Sensor name string.
 * @param c Sensor to serialize.
 * @param write_near_far Whether to emit near/far clip plane fields.
 */
template <typename T>
static void write_camera_like(std::ofstream& file, const std::string& kind, const char* name,
                              const T& c, bool write_near_far) {
    file << "\n        " << kind << " \"" << name << "\"\n        {\n";
    file << "            width = " << c.width << "\n";
    file << "            height = " << c.height << "\n";
    file << "            fov = " << c.fov_deg << "\n";
    file << "            rate = " << c.update_rate << "\n";
    if (write_near_far) {
        file << "            near = " << c.near_plane << "\n";
        file << "            far = " << c.far_plane << "\n";
    }
    if (c.position != Vec3{} || c.rotation != Vec3{}) {
        file << "            offset = ";
        write_vec3(file, c.position);
        file << "\n";
        file << "            rotation = ";
        write_vec3(file, c.rotation);
        file << "\n";
    }
    file << "        }\n";
}

bool WorldSerializer::save(const World& world, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        auto log = get_logger();
        log->error("Cannot write world file: {}", path);
        return false;
    }

    file << "world \"untitled\"\n{\n";

    if (world.has_terrain()) {
        std::string tpath = path + ".terrain";
        save_terrain(world.terrain(), tpath);
        file << "    terrain = \"" << std::filesystem::path(tpath).filename().string() << "\"\n";
    }

    {
        const SceneLighting& l = world.lighting();
        file << "    lighting\n    {\n";
        file << "        sun_direction = ";
        write_vec3(file, l.sun_direction);
        file << "\n";
        file << "        sun_color = ";
        write_vec3(file, l.sun_color);
        file << "\n";
        file << "        sun_intensity = " << l.sun_intensity << "\n";
        file << "        ambient_color = ";
        write_vec3(file, l.ambient_color);
        file << "\n";
        file << "        ambient_intensity = " << l.ambient_intensity << "\n";
        file << "        shadows = " << (l.shadows_enabled ? "true" : "false") << "\n";
        file << "    }\n";
    }

    {
        const Sky& sky = world.sky();
        file << "    sky\n    {\n";
        file << "        zenith_color = ";
        write_vec3(file, sky.zenith_color);
        file << "\n";
        file << "        horizon_color = ";
        write_vec3(file, sky.horizon_color);
        file << "\n";
        file << "    }\n";
    }

    {
        file << "    physics\n    {\n";
        file << "        gravity = " << world.gravity() << "\n";
        file << "    }\n";
    }

    size_t count = world.entities().max_allocated();
    for (size_t i = 1; i <= count; ++i) {
        Entity e = static_cast<Entity>(i);
        if (!world.valid(e)) continue;

        auto* name = world.get_component<Name>(e);
        auto* tf = world.get_component<Transform3D>(e);
        if (!tf) continue;

        std::string ename = name ? name->value : "entity_" + std::to_string(e);

        if (world.has_component<PointLight>(e)) {
            auto* light = world.get_component<PointLight>(e);
            file << "\n    light \"" << ename << "\"\n    {\n";
            file << "        position = ";
            write_vec3(file, tf->position);
            file << "\n";
            Vec3 euler = tf->rotation.to_euler();
            file << "        rotation = ";
            write_vec3(file, euler);
            file << "\n";
            file << "        color = ";
            write_vec3(file, light->color);
            file << "\n";
            file << "        intensity = " << light->intensity << "\n";
            file << "        range = " << light->range << "\n";
            file << "    }\n";
            continue;
        }

        if (world.has_component<DifferentialDrive>(e)) {
            auto* drive = world.get_component<DifferentialDrive>(e);
            file << "\n    robot \"" << ename << "\"\n    {\n";
            file << "        type = \"differential_drive\"\n";
            file << "        position = ";
            write_vec3(file, tf->position);
            file << "\n";
            Vec3 euler = tf->rotation.to_euler();
            file << "        rotation = ";
            write_vec3(file, euler);
            file << "\n";
            if (drive) {
                file << "        wheel_base = " << drive->wheel_base << "\n";
                file << "        max_speed = " << drive->max_linear_speed << "\n";
                file << "        odom_rate = " << drive->odom_rate << "\n";
                file << "        odom_noise = " << drive->odom_noise_stddev << "\n";
            }

            if (world.has_component<LidarSensor2D>(e)) {
                auto* l = world.get_component<LidarSensor2D>(e);
                file << "\n        lidar \"scan\"\n        {\n";
                file << "            range = " << l->range_max << "\n";
                double fov =
                    robcraft::engine::math::rad_to_deg(std::abs(l->angle_max - l->angle_min));
                file << "            fov = " << fov << "\n";
                file << "            rays = " << l->num_rays << "\n";
                file << "            rate = " << l->update_rate << "\n";
                file << "            noise = " << l->noise_stddev << "\n";
                if (l->position != Vec3{} || l->rotation != Vec3{}) {
                    file << "            offset = ";
                    write_vec3(file, l->position);
                    file << "\n";
                    file << "            rotation = ";
                    write_vec3(file, l->rotation);
                    file << "\n";
                }
                file << "        }\n";
            }
            if (world.has_component<CameraSensor>(e)) {
                auto* c = world.get_component<CameraSensor>(e);
                write_camera_like(file, "camera", "front_cam", *c, false);
            }
            if (world.has_component<DepthCameraSensor>(e)) {
                auto* c = world.get_component<DepthCameraSensor>(e);
                write_camera_like(file, "depth_camera", "depth", *c, true);
            }
            if (world.has_component<LidarSensor3D>(e)) {
                auto* l = world.get_component<LidarSensor3D>(e);
                file << "\n        lidar3d \"cloud\"\n        {\n";
                file << "            range = " << l->range_max << "\n";
                double hfov = robcraft::engine::math::rad_to_deg(
                    std::abs(l->horizontal_fov_max - l->horizontal_fov_min));
                file << "            hfov = " << hfov << "\n";
                double vfov = robcraft::engine::math::rad_to_deg(
                    std::abs(l->vertical_fov_max - l->vertical_fov_min));
                file << "            vfov = " << vfov << "\n";
                file << "            rays = " << l->horizontal_rays << "\n";
                file << "            beams = " << l->vertical_beams << "\n";
                file << "            rate = " << l->update_rate << "\n";
                file << "            noise = " << l->noise_stddev << "\n";
                if (l->position != Vec3{} || l->rotation != Vec3{}) {
                    file << "            offset = ";
                    write_vec3(file, l->position);
                    file << "\n";
                    file << "            rotation = ";
                    write_vec3(file, l->rotation);
                    file << "\n";
                }
                file << "        }\n";
            }
            if (world.has_component<ImuSensor>(e)) {
                auto* im = world.get_component<ImuSensor>(e);
                file << "\n        imu \"imu\"\n        {\n";
                file << "            rate = " << im->update_rate << "\n";
                file << "            linear_noise = " << im->linear_acceleration_noise_stddev
                     << "\n";
                file << "            angular_noise = " << im->angular_velocity_noise_stddev << "\n";
                if (im->position != Vec3{} || im->rotation != Vec3{}) {
                    file << "            offset = ";
                    write_vec3(file, im->position);
                    file << "\n";
                    file << "            rotation = ";
                    write_vec3(file, im->rotation);
                    file << "\n";
                }
                file << "        }\n";
            }
            if (world.has_component<GpsSensor>(e)) {
                auto* g = world.get_component<GpsSensor>(e);
                file << "\n        gps \"gps\"\n        {\n";
                file << "            rate = " << g->update_rate << "\n";
                file << "            noise = " << g->position_noise_stddev << "\n";
                if (g->position != Vec3{} || g->rotation != Vec3{}) {
                    file << "            offset = ";
                    write_vec3(file, g->position);
                    file << "\n";
                    file << "            rotation = ";
                    write_vec3(file, g->rotation);
                    file << "\n";
                }
                file << "        }\n";
            }
            if (world.has_component<MagnetometerSensor>(e)) {
                auto* m = world.get_component<MagnetometerSensor>(e);
                file << "\n        magnetometer \"mag\"\n        {\n";
                file << "            rate = " << m->update_rate << "\n";
                file << "            field = " << m->field_strength << "\n";
                file << "            declination = " << m->declination_deg << "\n";
                file << "            inclination = " << m->inclination_deg << "\n";
                file << "            noise = " << m->magnetic_field_noise_stddev << "\n";
                if (m->position != Vec3{} || m->rotation != Vec3{}) {
                    file << "            offset = ";
                    write_vec3(file, m->position);
                    file << "\n";
                    file << "            rotation = ";
                    write_vec3(file, m->rotation);
                    file << "\n";
                }
                file << "        }\n";
            }
            file << "    }\n";
        } else {
            auto* col = world.get_component<BoxCollider>(e);
            std::string mesh_type =
                name ? robcraft::renderer::mesh_label_for_name(name->value) : "cube";
            file << "\n    object \"" << ename << "\"\n    {\n";
            file << "        mesh = \"" << mesh_type << "\"\n";
            file << "        position = ";
            write_vec3(file, tf->position);
            file << "\n";
            Vec3 euler = tf->rotation.to_euler();
            file << "        rotation = ";
            write_vec3(file, euler);
            file << "\n";
            file << "        scale = ";
            write_vec3(file, tf->scale);
            file << "\n";
            if (col) file << "        collider = \"box\"\n";
            file << "    }\n";
        }
    }

    file << "}\n";
    return true;
}

}  // namespace robcraft::engine::world
