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

#include "robcraft/editor/command/entity_snapshot.hpp"

#include "robcraft/engine/world/world.hpp"

namespace robcraft::editor::command {

using namespace robcraft::engine::world;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;

std::unique_ptr<EntitySnapshot> EntitySnapshot::capture(const World& world, Entity e) {
    if (!world.valid(e)) return nullptr;
    auto snap = std::make_unique<EntitySnapshot>();
    snap->entity_ = e;
    if (auto* c = world.get_component<Name>(e)) snap->name_ = *c;
    if (auto* c = world.get_component<Transform3D>(e)) snap->transform_ = *c;
    if (auto* c = world.get_component<BoxCollider>(e)) snap->collider_ = *c;
    if (auto* c = world.get_component<DifferentialDrive>(e)) snap->drive_ = *c;
    if (auto* c = world.get_component<LidarSensor2D>(e)) snap->lidar_ = *c;
    if (auto* c = world.get_component<ImuSensor>(e)) snap->imu_ = *c;
    if (auto* c = world.get_component<GpsSensor>(e)) snap->gps_ = *c;
    if (auto* c = world.get_component<MagnetometerSensor>(e)) snap->magnetometer_ = *c;
    if (auto* c = world.get_component<CameraSensor>(e)) {
        snap->camera_ = *c;
        // Transient render output (~900 KB per robot), not meaningful state.
        snap->camera_->image_data.clear();
    }
    if (auto* c = world.get_component<DepthCameraSensor>(e)) {
        snap->depth_camera_ = *c;
        // Transient render output, not meaningful state.
        snap->depth_camera_->depth_data.clear();
    }
    if (auto* c = world.get_component<LidarSensor3D>(e)) snap->lidar3d_ = *c;
    if (auto* c = world.get_component<PointLight>(e)) snap->point_light_ = *c;
    return snap;
}

void EntitySnapshot::restore(World& world) {
    Entity e = this->entity_;
    if (!world.valid(e)) {
        // Recreate at the recorded id so cross-command undo/redo stays
        // consistent (a fresh create() would steal a free-list id).
        e = world.create_entity_at(e);
        this->entity_ = e;
    }

    world.remove_component<Name>(e);
    world.remove_component<Transform3D>(e);
    world.remove_component<BoxCollider>(e);
    world.remove_component<DifferentialDrive>(e);
    world.remove_component<LidarSensor2D>(e);
    world.remove_component<ImuSensor>(e);
    world.remove_component<GpsSensor>(e);
    world.remove_component<MagnetometerSensor>(e);
    world.remove_component<CameraSensor>(e);
    world.remove_component<DepthCameraSensor>(e);
    world.remove_component<LidarSensor3D>(e);
    world.remove_component<PointLight>(e);

    if (this->name_) world.add_component<Name>(e, *this->name_);
    if (this->transform_) world.add_component<Transform3D>(e, *this->transform_);
    if (this->collider_) world.add_component<BoxCollider>(e, *this->collider_);
    if (this->drive_) world.add_component<DifferentialDrive>(e, *this->drive_);
    if (this->lidar_) world.add_component<LidarSensor2D>(e, *this->lidar_);
    if (this->imu_) world.add_component<ImuSensor>(e, *this->imu_);
    if (this->gps_) world.add_component<GpsSensor>(e, *this->gps_);
    if (this->magnetometer_) world.add_component<MagnetometerSensor>(e, *this->magnetometer_);
    if (this->camera_) {
        CameraSensor cam = *this->camera_;
        cam.rebuild();
        world.add_component<CameraSensor>(e, cam);
    }
    if (this->depth_camera_) {
        DepthCameraSensor cam = *this->depth_camera_;
        cam.rebuild();
        world.add_component<DepthCameraSensor>(e, cam);
    }
    if (this->lidar3d_) world.add_component<LidarSensor3D>(e, *this->lidar3d_);
    if (this->point_light_) world.add_component<PointLight>(e, *this->point_light_);
}

}  // namespace robcraft::editor::command
