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

#pragma once

#include <memory>
#include <optional>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::editor::command {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;
using namespace robcraft::engine::world;

/** @brief Full component capture of one entity, restorable into a World. */
class EntitySnapshot {
public:
    /**
     * @brief Captures every registered component of an entity.
     * @param world The world containing the entity.
     * @param e The entity to capture.
     * @return A snapshot, or nullptr if the entity is invalid.
     */
    static std::unique_ptr<EntitySnapshot> capture(const World& world, Entity e);

    /**
     * @brief Restores this snapshot onto the entity, creating it if needed.
     * The entity is first stripped of all registered components so the result
     * is exact. If the entity had been destroyed, a new one is created and this
     * snapshot's id is updated to it so later undo/redo cycles stay consistent.
     * @param world The world to restore into.
     */
    void restore(World& world);

    /**
     * @brief Returns the entity id this snapshot captures.
     * @return The captured entity id (updated on restore after recreation).
     */
    Entity entity() const { return this->entity_; }

private:
    /** @brief The captured entity id (updated on restore after recreation). */
    Entity entity_ = INVALID_ENTITY;
    /** @brief Captured name component. */
    std::optional<Name> name_;
    /** @brief Captured transform component. */
    std::optional<Transform3D> transform_;
    /** @brief Captured collider component. */
    std::optional<BoxCollider> collider_;
    /** @brief Captured drive component. */
    std::optional<DifferentialDrive> drive_;
    /** @brief Captured LiDAR component. */
    std::optional<LidarSensor2D> lidar_;
    /** @brief Captured IMU component. */
    std::optional<ImuSensor> imu_;
    /** @brief Captured GPS component. */
    std::optional<GpsSensor> gps_;
    /** @brief Captured magnetometer component. */
    std::optional<MagnetometerSensor> magnetometer_;
    /** @brief Captured camera component (transient image buffer excluded). */
    std::optional<CameraSensor> camera_;
    /** @brief Captured point light component. */
    std::optional<PointLight> point_light_;
    /** @brief Captured depth camera component (transient depth buffer excluded). */
    std::optional<DepthCameraSensor> depth_camera_;
    /** @brief Captured 3D LiDAR component. */
    std::optional<LidarSensor3D> lidar3d_;
};

}  // namespace robcraft::editor::command
