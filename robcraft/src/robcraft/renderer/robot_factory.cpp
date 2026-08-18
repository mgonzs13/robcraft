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

#include "robcraft/renderer/robot_factory.hpp"

#include <string>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/ecs/vertical_motion.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/model.hpp"
#include "robcraft/renderer/model_paths.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::magnetometer;

Entity create_robot(World& world, const std::string& prefix, double wx, double wz,
                    const Model* model, bool include_lidar) {
    Entity e = world.create_entity();

    float ground = world.has_terrain() ? world.terrain().height_at_world(wx, wz) : 0.0f;
    const PlacementSpec* spec = placement_spec_for_prefix(prefix);

    Transform3D tf;
    tf.scale = spec ? Vec3(spec->base_scale.x, spec->base_scale.y, spec->base_scale.z)
                    : Vec3(1.0, 0.5, 1.0);
    float offset = placement_ground_offset(spec, model, tf.scale);
    tf.position = Vec3(wx, ground + offset, wz);
    world.add_component<Transform3D>(e, tf);
    world.add_component<Name>(e, Name{prefix + "_" + std::to_string(e)});

    DifferentialDrive drive;
    drive.max_linear_speed = 1.5;
    world.add_component<DifferentialDrive>(e, drive);

    world.add_component<VerticalMotion>(e, VerticalMotion{});

    world.add_component<BoxCollider>(e, BoxCollider{collider_half_extents(model, tf.scale)});

    if (include_lidar) {
        LidarSensor2D lidar;
        lidar.num_rays = 270;
        lidar.range_max = 10.0;
        lidar.update_rate = 15.0;
        lidar.rebuild_angles();
        world.add_component<LidarSensor2D>(e, lidar);
    }

    world.add_component<ImuSensor>(e, ImuSensor{});
    world.add_component<MagnetometerSensor>(e, MagnetometerSensor{});
    world.add_component<CameraSensor>(e, CameraSensor{});
    return e;
}

}  // namespace robcraft::renderer
