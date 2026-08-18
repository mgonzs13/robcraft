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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/collision/spatial_grid.hpp"
#include "robcraft/engine/core/app_menu.hpp"
#include "robcraft/engine/core/data_path.hpp"
#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/core/keyboard_control.hpp"
#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/core/random.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/scene_entities.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/ecs/vertical_motion.hpp"
#include "robcraft/engine/io/file_dialog.hpp"
#include "robcraft/engine/io/text_path_dialog.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/simulation/simulation_clock.hpp"
#include "robcraft/engine/world/gravity_step.hpp"
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/engine/world/terrain_mesh.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/engine/world/world_serializer.hpp"
#include "robcraft/renderer/animation_player.hpp"
#include "robcraft/renderer/fbo.hpp"
#include "robcraft/renderer/model.hpp"
#include "robcraft/renderer/model_paths.hpp"
#include "robcraft/renderer/pick.hpp"
#include "robcraft/renderer/primitive_meshes.hpp"
#include "robcraft/renderer/reflection.hpp"
#include "robcraft/renderer/renderer.hpp"
#include "robcraft/renderer/scene_render.hpp"
#include "robcraft/renderer/sky_render.hpp"
#include "robcraft/renderer/texture.hpp"
#include "robcraft/renderer/texture_pack.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/ros2/ros2_bridge.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

// GLFW/imgui must follow engine headers: glfw3.h pulls in gl.h before glew.h.
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

/**
 * @brief Selects an animation clip name from a robot's drive state.
 * @param linear Current linear speed in m/s.
 * @param angular Current angular speed in rad/s.
 * @return "Idle", "Walk", or "Run".
 */
static const char* robot_clip_for_state(double linear, double angular) {
    double speed = std::abs(linear) + std::abs(angular) * 0.5;
    if (speed < 0.05) return "Idle";
    if (speed < 0.8) return "Walk";
    return "Run";
}

static bool load_world(robcraft::engine::world::World& world, const std::string& path) {
    if (!robcraft::engine::world::WorldSerializer::load(world, path)) return false;
    if (!world.has_terrain()) world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));
    return true;
}

/**
 * @brief Advances a sensor's update timer and runs its update when due.
 * @tparam T Sensor component type.
 * @param store The sensor component store (may be null).
 * @param sim_dt Simulation timestep in seconds.
 * @param update Called as update(entity, sensor) when the timer is due.
 */
template <typename T, typename F>
void tick_sensor(robcraft::engine::ecs::ComponentStore<T>* store, double sim_dt, F&& update) {
    if (!store) return;
    for (auto& [entity, sensor] : *store) {
        sensor.time_since_update += sim_dt;
        if (sensor.time_since_update >= 1.0 / sensor.update_rate) {
            update(entity, sensor);
        }
    }
}

int main(int argc, char* argv[]) {
    int tex_arg = -1;
    bool want_help = false;
    bool publish_world_frame = false;
    bool publish_odom_tf = true;
    bool headless = false;
    std::string world_path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            want_help = true;
        } else if (a == "--texture-size" && i + 1 < argc) {
            tex_arg = std::atoi(argv[++i]);
        } else if (a == "--world-frame") {
            publish_world_frame = true;
        } else if (a == "--no-odom-tf") {
            publish_odom_tf = false;
        } else if (a == "--headless") {
            headless = true;
        } else if (world_path.empty()) {
            world_path = a;
        }
    }
    if (want_help) {
        std::printf(
            "Usage: robcraft [world.world] [options]\n"
            "  world.world           .world file to load (no world loaded by default)\n"
            "  --texture-size N      shared terrain/building texture size: 256, 512, or 1024\n"
            "                        (default 256)\n"
            "  --world-frame         publish the shared world (map) TF frame\n"
            "                        (default off)\n"
            "  --no-odom-tf          do not publish the odom -> base_footprint TF so an external\n"
            "                        odometry source (e.g. an EKF) can own it\n"
            "  --headless            run without a visible window (still needs a GL context)\n"
            "  --help, -h            show this help and exit\n");
        return 0;
    }

    if (!world_path.empty()) {
        world_path = robcraft::engine::core::resolve_data_path(world_path);
    }

    auto logger = robcraft::engine::core::init_logger();
    logger->info("RobCraft v1.0.0 — lightweight robotics simulator");

    double sim_rate = 100.0;
    double sim_dt = 1.0 / sim_rate;
    robcraft::engine::simulation::SimulationClock clock(sim_dt);
    robcraft::engine::core::Random rng(54321);

    robcraft::engine::world::World world;

    if (!world_path.empty()) {
        if (!load_world(world, world_path)) {
            logger->error("Failed to load world: {}", world_path);
            return 1;
        }
        logger->info("Loaded world: {}", world_path);
    } else {
        logger->info("No world loaded; starting empty");
    }

    logger->info(
        "Created {} entities ({} with LiDAR)", world.entities().max_allocated(),
        world.store<robcraft::sensors::lidar::LidarSensor2D>()
            ? static_cast<int>(world.store<robcraft::sensors::lidar::LidarSensor2D>()->size())
            : 0);

    robcraft::ros2::ROS2Bridge ros2_bridge;
    ros2_bridge.set_publish_world_frame(publish_world_frame);
    ros2_bridge.set_publish_odom_tf(publish_odom_tf);
    logger->info("ROS 2 bridge active — use 'ros2 topic list' to see topics");

    robcraft::renderer::Renderer renderer;
    int width = 1280;
    int height = 720;

    if (!renderer.init(width, height, "RobCraft", headless)) {
        logger->error("Failed to initialize renderer");
        return 1;
    }
    if (headless) logger->info("Running in headless mode (no visible window)");
    renderer.set_window_icon(robcraft::engine::core::resolve_data_path("assets/logo.png"));

    if (!headless) {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
        ImGui_ImplOpenGL3_Init("#version 330");
    }

    robcraft::renderer::PrimitiveMeshes meshes = robcraft::renderer::make_primitive_meshes();

    robcraft::renderer::ModelCache model_cache;
    std::unordered_map<robcraft::engine::core::Entity, robcraft::renderer::AnimationPlayer>
        robot_players;

    int tex_size = (tex_arg > 0) ? tex_arg : 256;
    robcraft::renderer::TexturePack app_tex;
    app_tex.load(tex_size);

    robcraft::renderer::SceneDrawContext scene_ctx{renderer.shader(), model_cache, app_tex, meshes};

    // Main-app skin lookup: per-entity AnimationPlayers updated by the sim loop.
    auto scene_skin = [&](robcraft::engine::core::Entity e,
                          const std::shared_ptr<robcraft::renderer::Model>&)
        -> const std::vector<robcraft::engine::math::Mat4>* {
        auto pit = robot_players.find(e);
        if (pit != robot_players.end() && pit->second.has_clip()) {
            return &pit->second.joint_matrices();
        }
        return nullptr;
    };

    auto draw_scene_entity = [&](robcraft::engine::core::Entity e) {
        robcraft::renderer::draw_scene_entity(scene_ctx, world, e, 1.0f, scene_skin);
    };

    double tex_repeat = 4.0;

    robcraft::renderer::Mesh terrain_mesh;
    robcraft::renderer::Mesh water_mesh;
    robcraft::renderer::FBO reflection_fbo;
    robcraft::renderer::FBO shadow_fbo;
    robcraft::engine::math::Mat4 sun_view_proj;

    std::vector<robcraft::engine::core::Entity> robot_entities;
    int controlled_entity_idx = 0;

    auto rebuild_scene = [&]() {
        world.terrain().set_texture_repeat(tex_repeat);
        robcraft::renderer::refit_world_colliders(world, model_cache);
        auto tdata = build_terrain_mesh(world.terrain());
        terrain_mesh.upload(tdata.vertices, tdata.indices, tdata.weights);
        auto wdata = build_terrain_water_mesh(world.terrain());
        if (wdata.vertices.empty()) {
            water_mesh.destroy();
        } else {
            water_mesh.upload(wdata.vertices, wdata.indices);
        }

        robot_entities.clear();
        robot_players.clear();
        auto* store = world.store<robcraft::robots::differential_drive::DifferentialDrive>();
        if (store) {
            for (auto& [e, _] : *store) robot_entities.push_back(e);
        }
        controlled_entity_idx = 0;

        double cs = world.terrain().cell_size();
        double hx = world.terrain().width() * cs * 0.5;
        double hz = world.terrain().height() * cs * 0.5;
        double d = std::max(hx, hz);
        renderer.camera().set_orbit_target(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
        renderer.camera().set_position(robcraft::engine::math::Vec3(d * 1.2, d * 1.0, d * 1.2));
        renderer.camera().look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

        ros2_bridge.init(world);
    };

    rebuild_scene();
    ros2_bridge.start_publishing();

    // Reloads the initial world: the file passed via CLI, else stays empty.
    auto reset_world = [&]() {
        std::lock_guard<std::mutex> world_lock(ros2_bridge.world_mutex());
        world.clear();
        world.set_terrain(robcraft::engine::world::Terrain{});
        world.set_lighting(robcraft::engine::lighting::SceneLighting{});
        if (!world_path.empty()) {
            if (!load_world(world, world_path)) {
                logger->error("Failed to reload world: {}", world_path);
                return;
            }
        }
        rebuild_scene();
        clock.reset();
        logger->info("World reset");
    };

    double lidar_log_timer = 0.0;
    bool world_dialog_open = false;
    bool tab_was_pressed = false;
    bool show_help = true;
    robcraft::engine::core::KeyboardDriveControl keyboard_control;
    robcraft::engine::core::AppMenuState menu_state;
    double last_wall = renderer.time();

    while (renderer.is_running() && ros2_bridge.ok()) {
        renderer.poll_events();

        GLFWwindow* win = glfwGetCurrentContext();

        double real_dt = renderer.delta_time();
        if (headless) {
            double now = renderer.time();
            real_dt = now - last_wall;
            last_wall = now;
        }
        // Windowed mode clamps a stall to one frame; headless must NOT clamp so
        // the accumulator keeps the sim synced to wall clock over long runs.
        if (!headless && real_dt > 0.1) real_dt = 0.016;

        if (!headless) {
            static bool right_prev = false;
            bool right_now = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            if (right_now && !right_prev && !ImGui::GetIO().WantCaptureMouse) {
                double mx, my;
                glfwGetCursorPos(win, &mx, &my);
                auto pivot = robcraft::renderer::pick_world_point(
                    renderer.camera(), world, mx, my, 0, 0, renderer.width(), renderer.height());
                if (!pivot && !renderer.camera().has_orbit_target()) {
                    pivot = robcraft::renderer::pick_point_on_plane(
                        renderer.camera(), mx, my, 0, 0, renderer.width(), renderer.height(), 0.0);
                }
                if (pivot) {
                    renderer.camera().set_orbit_target(*pivot);
                    renderer.camera().look_at(*pivot);
                }
            }
            right_prev = right_now;
        }

        if (!headless && !ImGui::GetIO().WantCaptureKeyboard) renderer.process_input(real_dt);

        ros2_bridge.spin_once();

        if (!headless) {
            if (!ImGui::GetIO().WantCaptureKeyboard) {
                robcraft::engine::core::KeyboardDriveCommand cmd =
                    keyboard_control.update(glfwGetKey(win, GLFW_KEY_I) == GLFW_PRESS,
                                            glfwGetKey(win, GLFW_KEY_K) == GLFW_PRESS,
                                            glfwGetKey(win, GLFW_KEY_J) == GLFW_PRESS,
                                            glfwGetKey(win, GLFW_KEY_L) == GLFW_PRESS,
                                            glfwGetKey(win, GLFW_KEY_U) == GLFW_PRESS);

                if (cmd.active && !robot_entities.empty()) {
                    auto* drive =
                        world
                            .get_component<robcraft::robots::differential_drive::DifferentialDrive>(
                                robot_entities[controlled_entity_idx]);
                    if (drive) {
                        double wb = drive->wheel_base;
                        drive->left_velocity = cmd.linear - cmd.angular * wb * 0.5;
                        drive->right_velocity = cmd.linear + cmd.angular * wb * 0.5;
                    }
                }

                bool tab_down = glfwGetKey(win, GLFW_KEY_TAB) == GLFW_PRESS;
                if (tab_down && !tab_was_pressed && !robot_entities.empty()) {
                    controlled_entity_idx =
                        (controlled_entity_idx + 1) % static_cast<int>(robot_entities.size());
                    logger->info("Controlling robot {}", controlled_entity_idx);
                }
                tab_was_pressed = tab_down;
            }
        }

        static bool o_prev = false;
        static bool r_prev = false;
        auto request_open_world = [&]() {
            if (robcraft::engine::io::native_dialog_available()) {
                if (auto p = robcraft::engine::io::native_open_file()) {
                    std::lock_guard<std::mutex> world_lock(ros2_bridge.world_mutex());
                    robcraft::engine::world::World next;
                    if (load_world(next, *p)) {
                        world = std::move(next);
                        rebuild_scene();
                        logger->info("Loaded world: {}", *p);
                    } else {
                        logger->error("Failed to load world: {}", *p);
                    }
                }
            } else {
                world_dialog_open = true;
            }
        };
        if (!headless) {
            bool ctrl = glfwGetKey(win, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                        glfwGetKey(win, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
            bool o_down = glfwGetKey(win, GLFW_KEY_O) == GLFW_PRESS;
            if (!ImGui::GetIO().WantCaptureKeyboard && ctrl && o_down && !o_prev) {
                request_open_world();
            }
            o_prev = ctrl && o_down;

            bool r_down = glfwGetKey(win, GLFW_KEY_R) == GLFW_PRESS;
            if (!ImGui::GetIO().WantCaptureKeyboard && ctrl && r_down && !r_prev) {
                reset_world();
            }
            r_prev = ctrl && r_down;
        }

        int ticks = clock.step(real_dt);
        {
            std::lock_guard<std::mutex> world_lock(ros2_bridge.world_mutex());
            for (int t = 0; t < ticks; ++t) {
                auto* dd_store =
                    world.store<robcraft::robots::differential_drive::DifferentialDrive>();
                if (dd_store) {
                    for (auto& [entity, dd] : *dd_store) {
                        auto* tf = world.get_component<robcraft::engine::ecs::Transform3D>(entity);
                        if (!tf) continue;

                        double v = dd.linear_velocity();
                        double omega = dd.angular_velocity();

                        if (omega != 0.0) {
                            auto rot = robcraft::engine::math::Quaternion::from_euler(
                                0.0, omega * sim_dt, 0.0);
                            tf->rotation = rot * tf->rotation;
                        }

                        robcraft::engine::math::Vec3 fwd =
                            tf->rotation.rotate(robcraft::engine::math::Vec3(0.0, 0.0, 1.0));
                        tf->position = tf->position + fwd * (v * sim_dt);
                        float ground_y =
                            world.terrain().height_at_world(tf->position.x, tf->position.z);
                        auto* name_comp = world.get_component<robcraft::engine::ecs::Name>(entity);
                        const robcraft::renderer::PlacementSpec* spec =
                            name_comp
                                ? robcraft::renderer::placement_spec_for_name(name_comp->value)
                                : nullptr;
                        float frac = spec ? spec->ground_frac : 0.5f;
                        double rest_y = ground_y + frac * tf->scale.y;
                        double y = tf->position.y;
                        if (auto* vm = world.get_component<robcraft::engine::ecs::VerticalMotion>(
                                entity)) {
                            robcraft::engine::world::apply_gravity(y, vm->vertical_velocity, rest_y,
                                                                   world.gravity(), sim_dt);
                        } else {
                            y = rest_y;
                        }
                        tf->position.y = y;

                        auto& player = robot_players[entity];
                        if (!player.has_clip()) {
                            std::string model_path;
                            if (name_comp)
                                model_path =
                                    robcraft::renderer::draw_model_path_for_name(name_comp->value);
                            if (!model_path.empty())
                                player.set_model(model_cache.get_skinned(model_path));
                        }
                        if (player.has_clip()) {
                            const char* clip =
                                robot_clip_for_state(dd.linear_velocity(), dd.angular_velocity());
                            if (player.current_name() != clip) player.play(clip);
                            player.update(sim_dt);
                        }
                    }
                }

                {
                    auto* tf_store = world.store<robcraft::engine::ecs::Transform3D>();
                    auto* name_store = world.store<robcraft::engine::ecs::Name>();
                    if (tf_store && name_store) {
                        size_t max_e = world.entities().max_allocated();
                        for (size_t i = 1; i <= max_e; ++i) {
                            robcraft::engine::core::Entity e =
                                static_cast<robcraft::engine::core::Entity>(i);
                            if (!world.valid(e)) continue;
                            if (world.has_component<
                                    robcraft::robots::differential_drive::DifferentialDrive>(e))
                                continue;
                            auto* tf = tf_store->get(e);
                            if (!tf) continue;
                            auto* name_comp = name_store->get(e);
                            if (!name_comp) continue;
                            std::string model_path =
                                robcraft::renderer::draw_model_path_for_name(name_comp->value);
                            if (model_path.empty()) continue;
                            if (model_path.rfind(".gltf") == std::string::npos &&
                                model_path.rfind(".glb") == std::string::npos)
                                continue;
                            auto& player = robot_players[e];
                            if (!player.has_clip()) {
                                player.set_model(model_cache.get_skinned(model_path));
                                if (player.has_clip()) player.play("Idle");
                            } else if (player.current_name() != "Idle") {
                                player.play("Idle");
                            }
                            player.update(sim_dt);
                        }
                    }
                }

                auto* col_store = world.store<robcraft::engine::collision::BoxCollider>();
                auto* tf_store = world.store<robcraft::engine::ecs::Transform3D>();
                if (col_store && tf_store) {
                    double cs = world.terrain().cell_size();
                    double hx = world.terrain().width() * cs * 0.5;
                    double hz = world.terrain().height() * cs * 0.5;
                    // query_aabb treats a box's max edge as exclusive while
                    // AABB::overlaps is inclusive; eps keeps boundary-touching
                    // pairs (which split across grid cells) in the broad phase.
                    double eps = cs * 1e-6;
                    robcraft::engine::collision::SpatialGrid grid(cs, -hx, -hz, hx, hz);
                    for (auto& [e, col] : *col_store) {
                        auto* tf = tf_store->get(e);
                        if (tf) grid.insert(e, tf->position, col, tf->rotation);
                    }
                    for (auto& [a, col_a] : *col_store) {
                        auto* tf_a = tf_store->get(a);
                        if (!tf_a) continue;
                        auto aabb_a = robcraft::engine::collision::AABB::from_box(
                            tf_a->position, col_a, tf_a->rotation);
                        robcraft::engine::collision::AABB query{
                            {aabb_a.min.x - eps, aabb_a.min.y, aabb_a.min.z - eps},
                            {aabb_a.max.x + eps, aabb_a.max.y, aabb_a.max.z + eps}};
                        for (robcraft::engine::core::Entity b : grid.query_aabb(query)) {
                            if (b <= a) continue;
                            auto* col_b = col_store->get(b);
                            auto* tf_b = tf_store->get(b);
                            if (!col_b || !tf_b) continue;
                            auto aabb_b = robcraft::engine::collision::AABB::from_box(
                                tf_b->position, *col_b, tf_b->rotation);
                            if (aabb_a.overlaps(aabb_b)) {
                                robcraft::engine::math::Vec3 vel_a;
                                robcraft::engine::math::Vec3 vel_b;
                                if (auto* dd =
                                        world.get_component<robcraft::robots::differential_drive::
                                                                DifferentialDrive>(a))
                                    vel_a = tf_a->rotation.rotate(robcraft::engine::math::Vec3(
                                        0.0, 0.0, dd->linear_velocity()));
                                if (auto* dd =
                                        world.get_component<robcraft::robots::differential_drive::
                                                                DifferentialDrive>(b))
                                    vel_b = tf_b->rotation.rotate(robcraft::engine::math::Vec3(
                                        0.0, 0.0, dd->linear_velocity()));
                                robcraft::engine::collision::resolve_overlap(
                                    aabb_a, aabb_b, vel_a, vel_b, tf_a->position, tf_b->position);
                            }
                        }
                    }
                }

                tick_sensor(world.store<robcraft::sensors::lidar::LidarSensor2D>(), sim_dt,
                            [&](robcraft::engine::core::Entity entity,
                                robcraft::sensors::lidar::LidarSensor2D& lidar) {
                                robcraft::sensors::lidar::lidar_update(entity, lidar, world, rng);
                            });
                tick_sensor(world.store<robcraft::sensors::lidar3d::LidarSensor3D>(), sim_dt,
                            [&](robcraft::engine::core::Entity entity,
                                robcraft::sensors::lidar3d::LidarSensor3D& lidar) {
                                robcraft::sensors::lidar3d::lidar3d_update(entity, lidar, world,
                                                                           rng);
                            });
                {
                    auto* dd_store2 =
                        world.store<robcraft::robots::differential_drive::DifferentialDrive>();
                    tick_sensor(
                        world.store<robcraft::sensors::imu::ImuSensor>(), sim_dt,
                        [&](robcraft::engine::core::Entity entity,
                            robcraft::sensors::imu::ImuSensor& imu) {
                            auto* tf =
                                world.get_component<robcraft::engine::ecs::Transform3D>(entity);
                            auto* dd = dd_store2 ? dd_store2->get(entity) : nullptr;
                            if (tf && dd) {
                                robcraft::engine::math::Vec3 world_vel = tf->rotation.rotate(
                                    robcraft::engine::math::Vec3(0.0, 0.0, dd->linear_velocity()));
                                robcraft::sensors::imu::imu_update(imu, world_vel, tf->rotation,
                                                                   dd->angular_velocity(),
                                                                   world.gravity(), sim_dt, rng);
                            }
                        });
                }
                tick_sensor(world.store<robcraft::sensors::gps::GpsSensor>(), sim_dt,
                            [&](robcraft::engine::core::Entity entity,
                                robcraft::sensors::gps::GpsSensor& gps) {
                                auto* tf =
                                    world.get_component<robcraft::engine::ecs::Transform3D>(entity);
                                if (tf) {
                                    robcraft::engine::math::Vec3 sample =
                                        tf->position + tf->rotation.rotate(gps.position);
                                    robcraft::sensors::gps::gps_update(gps, sample, rng);
                                }
                            });
                tick_sensor(
                    world.store<robcraft::sensors::magnetometer::MagnetometerSensor>(), sim_dt,
                    [&](robcraft::engine::core::Entity entity,
                        robcraft::sensors::magnetometer::MagnetometerSensor& mag) {
                        auto* tf = world.get_component<robcraft::engine::ecs::Transform3D>(entity);
                        if (tf) {
                            robcraft::sensors::magnetometer::magnetometer_update(mag, tf->rotation,
                                                                                 sim_dt, rng);
                        }
                    });
                tick_sensor(world.store<robcraft::sensors::camera::CameraSensor>(), sim_dt,
                            [](robcraft::engine::core::Entity,
                               robcraft::sensors::camera::CameraSensor&) {});
                tick_sensor(world.store<robcraft::sensors::depth_camera::DepthCameraSensor>(),
                            sim_dt,
                            [](robcraft::engine::core::Entity,
                               robcraft::sensors::depth_camera::DepthCameraSensor&) {});
            }
        }

        ros2_bridge.set_sim_time(clock.time(), clock.dt());

        lidar_log_timer += real_dt;
        if (lidar_log_timer >= 2.0) {
            lidar_log_timer = 0.0;
            auto* lidar_store = world.store<robcraft::sensors::lidar::LidarSensor2D>();
            if (lidar_store) {
                for (auto& [entity, lidar] : *lidar_store) {
                    double min_r = lidar.range_max;
                    double max_r = 0.0;
                    int finite_rays = 0;
                    for (int i = 0; i < lidar.num_rays; ++i) {
                        if (std::isinf(lidar.last_ranges[i])) continue;
                        min_r = std::min(min_r, lidar.last_ranges[i]);
                        max_r = std::max(max_r, lidar.last_ranges[i]);
                        ++finite_rays;
                    }
                    logger->info("LiDAR E{}: {} rays ({} detected), min={:.2f}m max={:.2f}m",
                                 entity, lidar.num_rays, finite_rays, min_r, max_r);
                }
            }
        }

        auto render_scene_for_camera = [&](const robcraft::engine::math::Mat4& proj,
                                           const robcraft::engine::math::Mat4& view,
                                           const robcraft::engine::math::Vec3& cam_pos,
                                           robcraft::engine::core::Entity skip_entity) {
            renderer.shader().use();
            renderer.set_lighting(world.lighting());
            renderer.set_point_lights(world);
            if (shadow_fbo.valid()) renderer.set_shadow_map(shadow_fbo.depth_tex(), sun_view_proj);
            renderer.set_water_defaults();
            renderer.shader().set_uniform("uAlpha", 1.0f);
            renderer.shader().set_uniform("uProjection", proj.ptr());
            renderer.shader().set_uniform("uView", view.ptr());
            renderer.shader().set_uniform("uCameraPos", static_cast<float>(cam_pos.x),
                                          static_cast<float>(cam_pos.y),
                                          static_cast<float>(cam_pos.z));

            auto m = robcraft::engine::math::Mat4::from_position_rotation(
                robcraft::engine::math::Vec3(0, 0, 0),
                robcraft::engine::math::Quaternion::identity());
            renderer.set_terrain_textures(app_tex.terrain_albedo, app_tex.terrain_normal,
                                          app_tex.use_splat);
            renderer.draw_entity(m, terrain_mesh);
            if (water_mesh.valid()) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthFunc(GL_LEQUAL);
                renderer.shader().set_uniform("uWater", 1);
                renderer.draw_entity(m, water_mesh);
                renderer.shader().set_uniform("uWater", 0);
                glDepthFunc(GL_LESS);
                glDisable(GL_BLEND);
            }
            renderer.shader().set_uniform("uUseTerrainTexture", 0);

            for (robcraft::engine::core::Entity e :
                 robcraft::engine::ecs::collect_scene_entities(world)) {
                if (e == skip_entity) continue;
                draw_scene_entity(e);
            }
        };

        // Renders a sensor camera into an FBO and reads back its output.
        // @param e Sensor entity.
        // @param sensor_pos Local mount offset.
        // @param sensor_rot Local mount rotation (euler, radians).
        // @param width, height, fov_deg, near_plane, far_plane Sensor projection params.
        // @param clear_color RGBA clear color for the FBO.
        // @param draw_sky Whether to draw the gradient sky behind the scene.
        // @param fbos Per-entity FBO cache.
        // @param readback Reads the FBO pixels into the sensor (called with the FBO bound).
        auto render_sensor_view =
            [&](robcraft::engine::core::Entity e, const robcraft::engine::math::Vec3& sensor_pos,
                const robcraft::engine::math::Vec3& sensor_rot, int width, int height,
                double fov_deg, double near_plane, double far_plane, const float clear_color[4],
                bool draw_sky,
                std::unordered_map<robcraft::engine::core::Entity, robcraft::renderer::FBO>& fbos,
                const std::function<void(robcraft::renderer::FBO&)>& readback) {
                auto* tf = world.get_component<robcraft::engine::ecs::Transform3D>(e);
                if (!tf) return;

                auto& fbo = fbos[e];
                if (!fbo.valid() || fbo.width() != width || fbo.height() != height) {
                    fbo.create(width, height);
                }
                if (!fbo.valid()) return;

                robcraft::engine::math::Mat4 cam_proj = robcraft::engine::math::Mat4::perspective(
                    static_cast<float>(robcraft::engine::math::deg_to_rad(fov_deg)),
                    static_cast<float>(width) / height, static_cast<float>(near_plane),
                    static_cast<float>(far_plane));

                robcraft::engine::math::Quaternion cam_rot =
                    robcraft::engine::math::Quaternion::from_euler(sensor_rot.x, sensor_rot.y,
                                                                   sensor_rot.z);
                robcraft::engine::math::Vec3 cam_pos =
                    tf->position + tf->rotation.rotate(sensor_pos);
                robcraft::engine::math::Vec3 forward = tf->rotation.rotate(
                    cam_rot.rotate(robcraft::engine::math::Vec3(0.0, 0.0, 1.0)));
                robcraft::engine::math::Vec3 cam_target = cam_pos + forward;
                robcraft::engine::math::Vec3 cam_up = tf->rotation.rotate(
                    cam_rot.rotate(robcraft::engine::math::Vec3(0.0, 1.0, 0.0)));
                robcraft::engine::math::Mat4 cam_view =
                    robcraft::engine::math::Mat4::look_at(cam_pos, cam_target, cam_up);

                fbo.bind();
                glDisable(GL_CULL_FACE);
                glViewport(0, 0, width, height);
                glClearColor(clear_color[0], clear_color[1], clear_color[2], clear_color[3]);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                renderer.shader().use();
                if (draw_sky)
                    robcraft::renderer::draw_sky_background(renderer.shader(), cam_proj,
                                                            world.sky());
                render_scene_for_camera(cam_proj, cam_view, cam_pos, e);

                readback(fbo);
                fbo.unbind();
            };

        {
            std::lock_guard<std::mutex> sensor_lock(ros2_bridge.world_mutex());
            auto* cam_store = world.store<robcraft::sensors::camera::CameraSensor>();
            if (cam_store) {
                // Headless note: the shadow-map pass lives in the (skipped) presentation block,
                // so sensor camera/depth renders have no directional shadows in headless mode.
                static std::unordered_map<robcraft::engine::core::Entity, robcraft::renderer::FBO>
                    fbos;
                for (auto& [entity, cam] : *cam_store) {
                    if (cam.time_since_update < 1.0 / cam.update_rate) continue;
                    const float clear[4] = {0.1f, 0.15f, 0.2f, 1.0f};
                    render_sensor_view(
                        entity, cam.position, cam.rotation, cam.width, cam.height, cam.fov_deg,
                        cam.near_plane, cam.far_plane, clear, true, fbos,
                        [&](robcraft::renderer::FBO& fbo) { fbo.read_pixels_rgb(cam.image_data); });
                    cam.time_since_update = 0.0;
                }
            }
        }

        {
            std::lock_guard<std::mutex> sensor_lock(ros2_bridge.world_mutex());
            auto* depth_store = world.store<robcraft::sensors::depth_camera::DepthCameraSensor>();
            if (depth_store) {
                static std::unordered_map<robcraft::engine::core::Entity, robcraft::renderer::FBO>
                    dfbos;
                for (auto& [entity, depth] : *depth_store) {
                    if (depth.time_since_update < 1.0 / depth.update_rate) continue;
                    const float clear[4] = {0.0f, 0.0f, 0.0f, 1.0f};
                    render_sensor_view(
                        entity, depth.position, depth.rotation, depth.width, depth.height,
                        depth.fov_deg, depth.near_plane, depth.far_plane, clear, false, dfbos,
                        [&](robcraft::renderer::FBO& fbo) {
                            fbo.read_pixels_depth(depth.depth_data);
                            const float far_plane = static_cast<float>(depth.far_plane);
                            for (float& z : depth.depth_data) {
                                z = robcraft::sensors::depth_camera::linearize_depth(
                                    static_cast<float>(depth.near_plane),
                                    static_cast<float>(depth.far_plane), z);
                                // Pixels with no surface (background / beyond the
                                // far plane) linearize to the far plane; report 0 so
                                // downstream consumers treat them as invalid depth
                                // instead of phantom far-range points.
                                if (z >= far_plane) z = 0.0f;
                            }
                        });
                    depth.time_since_update = 0.0;
                }
            }
        }

        if (!headless) {
            glViewport(0, 0, renderer.width(), renderer.height());
            glDisable(GL_CULL_FACE);
            glClearColor(0.7f, 0.8f, 0.9f, 1.0f);
            renderer.begin_frame();

            renderer.shader().use();
            robcraft::renderer::draw_sky_background(
                renderer.shader(), renderer.camera().projection_matrix(), world.sky());
            renderer.shader().set_uniform("uAlpha", 1.0f);
            renderer.shader().set_uniform("uProjection",
                                          renderer.camera().projection_matrix().ptr());
            renderer.shader().set_uniform("uView", renderer.camera().view_matrix().ptr());

            // Sun shadow map: render terrain + collider entities from the sun's view.
            {
                robcraft::renderer::render_shadow_pass(
                    renderer.shader(), world, terrain_mesh, renderer.camera(), shadow_fbo,
                    [&](robcraft::engine::core::Entity e) { draw_scene_entity(e); },
                    &sun_view_proj);
                renderer.set_lighting(world.lighting());
                renderer.set_point_lights(world);
                glViewport(0, 0, renderer.width(), renderer.height());
            }

            // Planar water reflection: mirrored scene into reflection_fbo.
            bool have_reflection = false;
            robcraft::engine::math::Mat4 refl_view;
            {
                robcraft::renderer::WaterReflection refl =
                    robcraft::renderer::render_reflection_pass(
                        scene_ctx, world, terrain_mesh, renderer.camera(), reflection_fbo,
                        renderer.width(), renderer.height(), water_mesh.valid(),
                        [&](robcraft::engine::core::Entity e) { draw_scene_entity(e); });
                have_reflection = refl.active;
                refl_view = refl.view;
            }

            {
                auto model = robcraft::engine::math::Mat4::from_position_rotation(
                    robcraft::engine::math::Vec3(0, 0, 0),
                    robcraft::engine::math::Quaternion::identity());
                renderer.set_terrain_textures(app_tex.terrain_albedo, app_tex.terrain_normal,
                                              app_tex.use_splat);
                renderer.draw_entity(model, terrain_mesh);
                if (water_mesh.valid()) {
                    robcraft::renderer::WaterParams wparams;
                    robcraft::renderer::draw_water_surface(scene_ctx, renderer.camera(), water_mesh,
                                                           reflection_fbo, refl_view,
                                                           have_reflection, wparams);
                }
                renderer.shader().set_uniform("uUseTerrainTexture", 0);
            }

            {
                for (robcraft::engine::core::Entity e :
                     robcraft::engine::ecs::collect_scene_entities(world)) {
                    draw_scene_entity(e);
                }
            }

            {
                static bool f1_prev = false;
                bool f1 = glfwGetKey(win, GLFW_KEY_F1) == GLFW_PRESS;
                if (f1 && !f1_prev) show_help = !show_help;
                f1_prev = f1;
            }

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            robcraft::engine::core::AppMenuResult menu_result =
                robcraft::engine::core::render_app_menu(menu_state, app_tex.size,
                                                        publish_world_frame);
            if (menu_result.action == robcraft::engine::core::AppAction::Open) {
                request_open_world();
            } else if (menu_result.action == robcraft::engine::core::AppAction::Reset) {
                reset_world();
            } else if (menu_result.action == robcraft::engine::core::AppAction::Exit) {
                glfwSetWindowShouldClose(win, GLFW_TRUE);
            }
            if (menu_result.texture_size != 0 && menu_result.texture_size != app_tex.size) {
                app_tex.load(menu_result.texture_size);
                logger->info("Reloaded textures at size {}", menu_result.texture_size);
            }
            if (menu_result.world_frame >= 0 &&
                menu_result.world_frame != (publish_world_frame ? 1 : 0)) {
                publish_world_frame = menu_result.world_frame != 0;
                ros2_bridge.set_publish_world_frame(publish_world_frame);
                logger->info("World frame publishing {}",
                             publish_world_frame ? "enabled" : "disabled");
            }
            if (world_dialog_open) {
                auto path = robcraft::engine::io::imgui_path_modal("Open World", world_dialog_open);
                if (path) {
                    std::lock_guard<std::mutex> world_lock(ros2_bridge.world_mutex());
                    robcraft::engine::world::World next;
                    if (load_world(next, *path)) {
                        world = std::move(next);
                        rebuild_scene();
                        logger->info("Loaded world: {}", *path);
                    } else {
                        logger->error("Failed to load world: {}", *path);
                        world_dialog_open = true;
                    }
                }
            }
            if (show_help) {
                const ImGuiWindowFlags flags =
                    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
                ImGui::SetNextWindowPos(ImVec2(renderer.width() - 330.0f, 30.0f), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.6f);
                ImGui::Begin("Camera Help", nullptr, flags);
                ImGui::Text("WASD/QE  pan / up-down");
                ImGui::Text("Right-drag  orbit around clicked point");
                ImGui::Text("Scroll  zoom");
                ImGui::Text("I/K  forward/back   J/L  turn   U  stop");
                ImGui::Text("Tab  switch robot");
                ImGui::Text("Ctrl+O  open world    Ctrl+R  reset    F1  hide help");
                ImGui::End();
            }
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            renderer.end_frame();
        }

        // Pacing sleep: keeps the headless loop from becoming a busy spin. Use a
        // sub-millisecond value so the loop runs well above 200 Hz (a 1 ms sleep
        // wakes late under load and would cap the loop near 170 Hz, starving the
        // 200 Hz IMU publish rate).
        if (headless) std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (!headless) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    logger->info("Engine shutting down cleanly.");
    return 0;
}
