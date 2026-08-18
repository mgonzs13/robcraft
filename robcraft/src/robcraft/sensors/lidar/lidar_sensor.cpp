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

#include "robcraft/sensors/lidar/lidar_sensor.hpp"

#include <algorithm>
#include <cmath>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/sensors/lidar/lidar_raycast.hpp"

namespace robcraft::sensors::lidar {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;

LidarSensor2D::LidarSensor2D() {
    this->update_rate = 15.0;
    this->position = Vec3(0.0, 0.15, 0.0);
    this->rebuild_angles();
}

void LidarSensor2D::rebuild_angles() {
    if (this->num_rays <= 0) {
        this->last_angles.clear();
        this->last_ranges.clear();
        return;
    }

    this->last_angles.resize(this->num_rays);
    this->last_ranges.resize(this->num_rays, this->range_max);
    if (this->num_rays == 1) {
        this->last_angles[0] = (this->angle_min + this->angle_max) * 0.5;
        return;
    }

    double step = (this->angle_max - this->angle_min) / (this->num_rays - 1);
    for (int i = 0; i < this->num_rays; ++i) {
        this->last_angles[i] = this->angle_min + i * step;
    }
}

void lidar_update(Entity self_entity, LidarSensor2D& sensor, World& world, Random& rng) {
    auto* tf_store = world.store<Transform3D>();
    auto* col_store = world.store<BoxCollider>();
    if (!tf_store || !col_store) return;

    auto* self_tf = tf_store->get(self_entity);
    if (!self_tf) return;

    Quaternion sensor_rot =
        Quaternion::from_euler(sensor.rotation.x, sensor.rotation.y, sensor.rotation.z);
    Vec3 sensor_pos = self_tf->position + self_tf->rotation.rotate(sensor.position);

    // Spatial grid over the terrain footprint (falls back to 50x50 without
    // terrain) so rays keep detecting walls after the robot leaves the
    // 25 m origin-centered box.
    double cs = 2.0;
    double hx = 25.0, hz = 25.0;
    if (world.has_terrain()) {
        cs = world.terrain().cell_size();
        hx = world.terrain().width() * cs * 0.5;
        hz = world.terrain().height() * cs * 0.5;
    }

    LidarScene scene(world, self_entity, cs, hx, hz, true);

    for (int i = 0; i < sensor.num_rays; ++i) {
        double angle = sensor.last_angles[i];

        Vec3 dir_local(std::sin(angle), 0.0, std::cos(angle));
        Vec3 dir_world = self_tf->rotation.rotate(sensor_rot.rotate(dir_local));

        double closest =
            scene.raycast_hit(sensor_pos, dir_world, sensor.range_min, sensor.range_max, false);

        if (std::isinf(closest)) {
            // No detection: report +inf so consumers don't treat the lidar's
            // max range as a phantom obstacle; noise must not perturb it.
            sensor.last_ranges[i] = closest;
            continue;
        }

        double noise = rng.gaussian(0.0, sensor.noise_stddev);
        closest += noise;
        closest = std::clamp(closest, sensor.range_min, sensor.range_max);

        sensor.last_ranges[i] = closest;
    }

    sensor.time_since_update = 0.0;
}

}  // namespace robcraft::sensors::lidar
