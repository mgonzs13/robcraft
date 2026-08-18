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

#include "robcraft/editor/editor_inspector.hpp"

#include <imgui.h>

#include <cstring>
#include <string>
#include <vector>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;
using namespace robcraft::robots::differential_drive;

EditorInspector::EditorInspector(EditorApp& app) : app_(app) {}

void EditorInspector::render_inspector() {
    ImGui::Begin("Inspector", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);
    Entity e = this->app_.placement_.primary_selection();
    if (e == INVALID_ENTITY || !this->app_.world_.valid(e)) {
        ImGui::Text("No entity selected");
        ImGui::End();
        return;
    }
    if (this->app_.world_.valid(e) &&
        (!this->app_.inspector_before_ || this->app_.inspector_before_->entity() != e)) {
        this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
    }
    auto* nm = this->app_.world_.get_component<Name>(e);
    if (nm) {
        char b[256];
        std::strncpy(b, nm->value.c_str(), 255);
        b[255] = 0;
        if (ImGui::InputText("Name", b, 256)) {
            nm->value = b;
            this->app_.tools_.mark_modified();
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
    }
    if (auto* tf = this->app_.world_.get_component<Transform3D>(e)) {
        if (ImGui::CollapsingHeader("Transform")) {
            float p[3] = {(float)tf->position.x, (float)tf->position.y, (float)tf->position.z};
            if (ImGui::DragFloat3("Position", p, 0.1f)) {
                tf->position = Vec3(p[0], p[1], p[2]);
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            Vec3 euler = tf->rotation.to_euler();
            float rot[3] = {static_cast<float>(robcraft::engine::math::rad_to_deg(euler.x)),
                            static_cast<float>(robcraft::engine::math::rad_to_deg(euler.y)),
                            static_cast<float>(robcraft::engine::math::rad_to_deg(euler.z))};
            if (ImGui::DragFloat3("Rotation (deg)", rot, 1.0f)) {
                tf->rotation = Quaternion::from_euler(
                    static_cast<float>(robcraft::engine::math::deg_to_rad(rot[0])),
                    static_cast<float>(robcraft::engine::math::deg_to_rad(rot[1])),
                    static_cast<float>(robcraft::engine::math::deg_to_rad(rot[2])));
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            if (this->app_.gizmo_mode_ == GizmoMode::Rotate ||
                (this->app_.gizmo_drag_active_ && this->app_.gizmo_handle_ == GizmoHandle::Yaw)) {
                Vec3 euler2 = tf->rotation.to_euler();
                ImGui::Text("Yaw: %.1f deg",
                            static_cast<float>(robcraft::engine::math::rad_to_deg(euler2.y)));
            }
            float s[3] = {(float)tf->scale.x, (float)tf->scale.y, (float)tf->scale.z};
            if (ImGui::DragFloat3("Scale", s, 0.1f, 0.01f, 100.f)) {
                tf->scale = Vec3(s[0], s[1], s[2]);
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                this->app_.commit_inspector_edit(e);
                // Keep the collider (and the selection box drawn from it) in
                // sync with the rescaled model.
                robcraft::renderer::refit_world_colliders(this->app_.world_,
                                                          this->app_.model_cache_);
            }
        }
    }
    if (auto* c = this->app_.world_.get_component<BoxCollider>(e)) {
        if (ImGui::CollapsingHeader("BoxCollider")) {
            float he[3] = {(float)c->half_extents.x, (float)c->half_extents.y,
                           (float)c->half_extents.z};
            if (ImGui::DragFloat3("HalfExt", he, 0.05f, 0.01f, 100.f)) {
                c->half_extents = Vec3(he[0], he[1], he[2]);
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
        }
    }
    if (auto* pl = this->app_.world_.get_component<PointLight>(e)) {
        if (ImGui::CollapsingHeader("PointLight")) {
            float col[3] = {static_cast<float>(pl->color.x), static_cast<float>(pl->color.y),
                            static_cast<float>(pl->color.z)};
            if (ImGui::InputFloat3("Color", col)) {
                pl->color = Vec3(col[0], col[1], col[2]);
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            float it = pl->intensity;
            if (ImGui::InputFloat("Intensity", &it, 0.0f, 0.0f, "%.2f")) {
                pl->intensity = it;
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            float rng = pl->range;
            if (ImGui::InputFloat("Range", &rng, 0.0f, 0.0f, "%.2f")) {
                pl->range = rng;
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            if (ImGui::Button("Remove Point Light")) {
                this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                this->app_.world_.remove_component<PointLight>(e);
                this->app_.tools_.mark_modified();
                this->app_.commit_inspector_edit(e);
            }
        }
    }
    if (auto* dd = this->app_.world_.get_component<DifferentialDrive>(e)) {
        if (ImGui::CollapsingHeader("Drive")) {
            float wb = (float)dd->wheel_base;
            if (ImGui::DragFloat("WheelBase", &wb, 0.01f, 0.01f, 10.f)) {
                dd->wheel_base = wb;
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            float ms = (float)dd->max_linear_speed;
            if (ImGui::DragFloat("MaxSpeed", &ms, 0.1f, 0.1f, 100.f)) {
                dd->max_linear_speed = ms;
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            float orate = (float)dd->odom_rate;
            if (ImGui::DragFloat("Odom Rate", &orate, 0.5f, 0.1f, 500.f)) {
                dd->odom_rate = orate;
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
            float on = (float)dd->odom_noise_stddev;
            if (ImGui::DragFloat("Odom Noise", &on, 0.01f, 0.0f, 5.f)) {
                dd->odom_noise_stddev = on;
                this->app_.tools_.mark_modified();
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
        }
    }
    if (this->app_.world_.has_component<DifferentialDrive>(e)) {
        if (ImGui::CollapsingHeader("Sensors")) {
            if (auto* l = this->app_.world_.get_component<LidarSensor2D>(e)) {
                if (ImGui::TreeNode("LiDAR")) {
                    int rays = l->num_rays;
                    if (ImGui::DragInt("Rays", &rays, 1, 1, 1000)) {
                        l->num_rays = rays;
                        l->rebuild_angles();
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float r = (float)l->range_max;
                    if (ImGui::DragFloat("Range", &r, 0.1f, 0.1f, 100.f)) {
                        l->range_max = r;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float rt = (float)l->update_rate;
                    if (ImGui::DragFloat("Rate", &rt, 0.5f, 0.1f, 100.f)) {
                        l->update_rate = rt;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float ln = (float)l->noise_stddev;
                    if (ImGui::DragFloat("Noise", &ln, 0.01f, 0.0f, 10.f)) {
                        l->noise_stddev = ln;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::Button("Remove LiDAR")) {
                        this->app_.inspector_before_ =
                            EntitySnapshot::capture(this->app_.world_, e);
                        this->app_.world_.remove_component<LidarSensor2D>(e);
                        this->app_.tools_.mark_modified();
                        this->app_.commit_inspector_edit(e);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Button("Add LiDAR")) {
                    this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                    LidarSensor2D li;
                    li.num_rays = 270;
                    li.range_max = 10.0;
                    li.update_rate = 15.0;
                    li.rebuild_angles();
                    this->app_.world_.add_component<LidarSensor2D>(e, li);
                    this->app_.tools_.mark_modified();
                    this->app_.commit_inspector_edit(e);
                }
            }
            if (auto* ca = this->app_.world_.get_component<CameraSensor>(e)) {
                if (ImGui::TreeNode("Camera")) {
                    if (ImGui::DragInt("W", &ca->width, 1, 1, 4096)) {
                        ca->rebuild();
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::DragInt("H", &ca->height, 1, 1, 4096)) {
                        ca->rebuild();
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float f = (float)ca->fov_deg;
                    if (ImGui::DragFloat("FOV", &f, 1.f, 1.f, 179.f)) {
                        ca->fov_deg = f;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float crt = (float)ca->update_rate;
                    if (ImGui::DragFloat("Rate", &crt, 0.5f, 0.1f, 500.f)) {
                        ca->update_rate = crt;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::Button("Remove Camera")) {
                        this->app_.inspector_before_ =
                            EntitySnapshot::capture(this->app_.world_, e);
                        this->app_.world_.remove_component<CameraSensor>(e);
                        this->app_.tools_.mark_modified();
                        this->app_.commit_inspector_edit(e);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Button("Add Camera")) {
                    this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                    CameraSensor ca;
                    ca.width = 640;
                    ca.height = 480;
                    ca.update_rate = 30.0;
                    ca.rebuild();
                    this->app_.world_.add_component<CameraSensor>(e, ca);
                    this->app_.tools_.mark_modified();
                    this->app_.commit_inspector_edit(e);
                }
            }
            if (auto* im = this->app_.world_.get_component<ImuSensor>(e)) {
                if (ImGui::TreeNode("IMU")) {
                    float an = (float)im->angular_velocity_noise_stddev;
                    if (ImGui::DragFloat("Angular Noise", &an, 0.01f, 0.0f, 10.f)) {
                        im->angular_velocity_noise_stddev = an;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float aln = (float)im->linear_acceleration_noise_stddev;
                    if (ImGui::DragFloat("Linear Noise", &aln, 0.01f, 0.0f, 10.f)) {
                        im->linear_acceleration_noise_stddev = aln;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float rt = (float)im->update_rate;
                    if (ImGui::DragFloat("Rate", &rt, 0.5f, 0.1f, 500.f)) {
                        im->update_rate = rt;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::Button("Remove IMU")) {
                        this->app_.inspector_before_ =
                            EntitySnapshot::capture(this->app_.world_, e);
                        this->app_.world_.remove_component<ImuSensor>(e);
                        this->app_.tools_.mark_modified();
                        this->app_.commit_inspector_edit(e);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Button("Add IMU")) {
                    this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                    this->app_.world_.add_component<ImuSensor>(e, ImuSensor{});
                    this->app_.tools_.mark_modified();
                    this->app_.commit_inspector_edit(e);
                }
            }
            if (auto* g = this->app_.world_.get_component<GpsSensor>(e)) {
                if (ImGui::TreeNode("GPS")) {
                    float rt = (float)g->update_rate;
                    if (ImGui::DragFloat("Rate", &rt, 0.5f, 0.1f, 100.f)) {
                        g->update_rate = rt;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float gn = (float)g->position_noise_stddev;
                    if (ImGui::DragFloat("Noise", &gn, 0.01f, 0.0f, 10.f)) {
                        g->position_noise_stddev = gn;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::Button("Remove GPS")) {
                        this->app_.inspector_before_ =
                            EntitySnapshot::capture(this->app_.world_, e);
                        this->app_.world_.remove_component<GpsSensor>(e);
                        this->app_.tools_.mark_modified();
                        this->app_.commit_inspector_edit(e);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Button("Add GPS")) {
                    this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                    this->app_.world_.add_component<GpsSensor>(e, GpsSensor{});
                    this->app_.tools_.mark_modified();
                    this->app_.commit_inspector_edit(e);
                }
            }
            if (auto* m = this->app_.world_.get_component<MagnetometerSensor>(e)) {
                if (ImGui::TreeNode("Magnetometer")) {
                    float fs = (float)m->field_strength;
                    if (ImGui::DragFloat("Field", &fs, 0.5f, 0.1f, 500.f)) {
                        m->field_strength = fs;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float dec = (float)m->declination_deg;
                    if (ImGui::DragFloat("Declination", &dec, 0.5f, -180.f, 180.f)) {
                        m->declination_deg = dec;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float inc = (float)m->inclination_deg;
                    if (ImGui::DragFloat("Inclination", &inc, 0.5f, -90.f, 90.f)) {
                        m->inclination_deg = inc;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float rt = (float)m->update_rate;
                    if (ImGui::DragFloat("Rate", &rt, 0.5f, 0.1f, 500.f)) {
                        m->update_rate = rt;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float no = (float)m->magnetic_field_noise_stddev;
                    if (ImGui::DragFloat("Noise", &no, 0.01f, 0.f, 100.f)) {
                        m->magnetic_field_noise_stddev = no;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::Button("Remove Magnetometer")) {
                        this->app_.inspector_before_ =
                            EntitySnapshot::capture(this->app_.world_, e);
                        this->app_.world_.remove_component<MagnetometerSensor>(e);
                        this->app_.tools_.mark_modified();
                        this->app_.commit_inspector_edit(e);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Button("Add Magnetometer")) {
                    this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                    this->app_.world_.add_component<MagnetometerSensor>(e, MagnetometerSensor{});
                    this->app_.tools_.mark_modified();
                    this->app_.commit_inspector_edit(e);
                }
            }
            if (auto* dc = this->app_.world_.get_component<DepthCameraSensor>(e)) {
                if (ImGui::TreeNode("Depth Camera")) {
                    int dw = dc->width;
                    if (ImGui::DragInt("W", &dw, 1, 1, 4096)) {
                        dc->width = dw;
                        dc->rebuild();
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    int dh = dc->height;
                    if (ImGui::DragInt("H", &dh, 1, 1, 4096)) {
                        dc->height = dh;
                        dc->rebuild();
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float f = (float)dc->fov_deg;
                    if (ImGui::DragFloat("FOV", &f, 1.f, 1.f, 179.f)) {
                        dc->fov_deg = f;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float drt = (float)dc->update_rate;
                    if (ImGui::DragFloat("Rate", &drt, 0.5f, 0.1f, 500.f)) {
                        dc->update_rate = drt;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::Button("Remove Depth Camera")) {
                        this->app_.inspector_before_ =
                            EntitySnapshot::capture(this->app_.world_, e);
                        this->app_.world_.remove_component<DepthCameraSensor>(e);
                        this->app_.tools_.mark_modified();
                        this->app_.commit_inspector_edit(e);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Button("Add Depth Camera")) {
                    this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                    DepthCameraSensor dc;
                    dc.width = 640;
                    dc.height = 480;
                    dc.update_rate = 30.0;
                    dc.rebuild();
                    this->app_.world_.add_component<DepthCameraSensor>(e, dc);
                    this->app_.tools_.mark_modified();
                    this->app_.commit_inspector_edit(e);
                }
            }
            if (auto* l3 = this->app_.world_.get_component<LidarSensor3D>(e)) {
                if (ImGui::TreeNode("3D LiDAR")) {
                    int rays = l3->horizontal_rays;
                    if (ImGui::DragInt("Rays", &rays, 1, 1, 2048)) {
                        l3->horizontal_rays = rays;
                        l3->rebuild();
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    int beams = l3->vertical_beams;
                    if (ImGui::DragInt("Beams", &beams, 1, 1, 128)) {
                        l3->vertical_beams = beams;
                        l3->rebuild();
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float r = (float)l3->range_max;
                    if (ImGui::DragFloat("Range", &r, 0.1f, 0.1f, 200.f)) {
                        l3->range_max = r;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float rt = (float)l3->update_rate;
                    if (ImGui::DragFloat("Rate", &rt, 0.5f, 0.1f, 100.f)) {
                        l3->update_rate = rt;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    float l3n = (float)l3->noise_stddev;
                    if (ImGui::DragFloat("Noise", &l3n, 0.01f, 0.0f, 10.f)) {
                        l3->noise_stddev = l3n;
                        this->app_.tools_.mark_modified();
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) this->app_.commit_inspector_edit(e);
                    if (ImGui::Button("Remove 3D LiDAR")) {
                        this->app_.inspector_before_ =
                            EntitySnapshot::capture(this->app_.world_, e);
                        this->app_.world_.remove_component<LidarSensor3D>(e);
                        this->app_.tools_.mark_modified();
                        this->app_.commit_inspector_edit(e);
                    }
                    ImGui::TreePop();
                }
            } else {
                if (ImGui::Button("Add 3D LiDAR")) {
                    this->app_.inspector_before_ = EntitySnapshot::capture(this->app_.world_, e);
                    LidarSensor3D l3;
                    l3.rebuild();
                    this->app_.world_.add_component<LidarSensor3D>(e, l3);
                    this->app_.tools_.mark_modified();
                    this->app_.commit_inspector_edit(e);
                }
            }
        }
    }
    ImGui::Separator();
    std::string label = "Delete Selection";
    if (!this->app_.selection_.empty())
        label += " (" + std::to_string(this->app_.selection_.size()) + ")";
    if (ImGui::Button(label.c_str(), ImVec2(-1, 0))) this->app_.placement_.delete_selection();
    ImGui::End();
}

}  // namespace robcraft::editor
