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

#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/sensors/lidar/lidar_raycast.hpp"

namespace robcraft::sensors::lidar3d {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using namespace robcraft::sensors::lidar;

LidarSensor3D::LidarSensor3D() {
    this->update_rate = 15.0;
    this->position = Vec3(0.0, 0.15, 0.0);
    this->rebuild();
}

void LidarSensor3D::rebuild() {
    // Initialize as "no detection" (+inf) instead of range_max so the very
    // first cloud before any raycast is not a phantom shell of finite hits.
    this->last_ranges.resize(static_cast<size_t>(this->vertical_beams) * this->horizontal_rays,
                             std::numeric_limits<float>::infinity());
    this->last_dirs.resize(static_cast<size_t>(this->vertical_beams) * this->horizontal_rays);
    for (int b = 0; b < this->vertical_beams; ++b) {
        double el =
            (this->vertical_beams > 1)
                ? this->vertical_fov_min + b * (this->vertical_fov_max - this->vertical_fov_min) /
                                               (this->vertical_beams - 1)
                : this->vertical_fov_min;
        for (int r = 0; r < this->horizontal_rays; ++r) {
            double az = (this->horizontal_rays > 1)
                            ? this->horizontal_fov_min +
                                  r * (this->horizontal_fov_max - this->horizontal_fov_min) /
                                      (this->horizontal_rays - 1)
                            : this->horizontal_fov_min;
            Vec3 dir(std::sin(az) * std::cos(el), std::sin(el), std::cos(az) * std::cos(el));
            this->last_dirs[static_cast<size_t>(b) * this->horizontal_rays + r] = dir;
        }
    }
}

void lidar3d_update(Entity self_entity, LidarSensor3D& sensor, World& world, Random& rng) {
    auto* tf_store = world.store<Transform3D>();
    auto* col_store = world.store<BoxCollider>();
    if (!tf_store || !col_store) return;

    auto* self_tf = tf_store->get(self_entity);
    if (!self_tf) return;

    // Spatial grid over the terrain footprint (falls back to 50x50 without terrain).
    double cs = 2.0;
    double hx = 25.0, hz = 25.0;
    if (world.has_terrain()) {
        cs = world.terrain().cell_size();
        hx = world.terrain().width() * cs * 0.5;
        hz = world.terrain().height() * cs * 0.5;
    }

    Quaternion sensor_rot =
        Quaternion::from_euler(sensor.rotation.x, sensor.rotation.y, sensor.rotation.z);
    Vec3 sensor_pos = self_tf->position + self_tf->rotation.rotate(sensor.position);

    LidarScene scene(world, self_entity, cs, hx, hz, true);

    size_t total = static_cast<size_t>(sensor.vertical_beams) * sensor.horizontal_rays;
    for (size_t i = 0; i < total; ++i) {
        Vec3 dir_local = sensor.last_dirs[i];
        Vec3 dir_world = self_tf->rotation.rotate(sensor_rot.rotate(dir_local));

        double closest =
            scene.raycast_hit(sensor_pos, dir_world, sensor.range_min, sensor.range_max, true);

        if (std::isinf(closest)) {
            // No detection: report +inf so consumers don't treat the lidar's
            // max range as a phantom obstacle; noise must not perturb it.
            sensor.last_ranges[i] = static_cast<float>(closest);
            continue;
        }

        double noise = rng.gaussian(0.0, sensor.noise_stddev);
        closest += noise;
        closest = std::clamp(closest, sensor.range_min, sensor.range_max);

        sensor.last_ranges[i] = static_cast<float>(closest);
    }

    sensor.time_since_update = 0.0;
}

}  // namespace robcraft::sensors::lidar3d
