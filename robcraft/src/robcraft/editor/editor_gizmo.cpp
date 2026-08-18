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

#include "robcraft/editor/editor_gizmo.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/math/constants.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::math;
using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::robots::differential_drive;

namespace {

constexpr double kPi = robcraft::engine::math::kPi;

// The rotate ring's radius relative to the move arrows' length, so the ring
// reads as clearly wider than the arrows (Unity-style).
constexpr double kRingScale = 1.6;
// Inner radius of the ring annulus, as a fraction of its outer radius, so the
// ring renders as a filled surface rather than a thin line.
constexpr double kRingInner = 0.72;

// Builds a solid arrow along +axis: a box shaft (0 -> 0.78) plus a 6-sided cone
// head (0.78 -> 1.0), all baked in the given color.
Mesh build_arrow_mesh(const Vec3& axis, float r, float g, float b) {
    Vec3 u = (std::abs(axis.y) < 0.9) ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    u = axis.cross(u).normalized();
    Vec3 v = u.cross(axis).normalized();

    const double hw = 0.03;
    const double head_start = 0.78;
    const int seg = 6;

    std::vector<Vertex> verts;
    std::vector<GLuint> idx;
    auto push = [&](const Vec3& p) {
        verts.push_back({(float)p.x, (float)p.y, (float)p.z, 0.0f, 1.0f, 0.0f, r, g, b, 0.0f, 0.0f,
                         1.0f, 0.0f, 0.0f});
    };

    const Vec3 perp[4] = {u, v, u * -1.0, v * -1.0};
    for (int i = 0; i < 4; ++i) {
        Vec3 a = perp[i] * hw;
        Vec3 b = perp[(i + 1) % 4] * hw;
        unsigned int base = static_cast<unsigned int>(verts.size());
        push(a);
        push(b);
        push(axis * head_start + a);
        push(axis * head_start + b);
        idx.push_back(base);
        idx.push_back(base + 1);
        idx.push_back(base + 2);
        idx.push_back(base + 2);
        idx.push_back(base + 1);
        idx.push_back(base + 3);
    }

    unsigned int tip = static_cast<unsigned int>(verts.size());
    push(axis * 1.0);
    for (int i = 0; i < seg; ++i) {
        double ang = 2.0 * kPi * i / seg;
        Vec3 dir = u * std::cos(ang) + v * std::sin(ang);
        push(axis * head_start + dir * hw * 1.8);
    }
    for (int i = 0; i < seg; ++i) {
        idx.push_back(tip);
        idx.push_back(tip + 1 + i);
        idx.push_back(tip + 1 + (i + 1) % seg);
    }

    Mesh m;
    m.upload(verts, idx);
    return m;
}

// Builds a flat annulus (filled ring) in the XZ plane: outer radius 1.0, inner
// radius kRingInner, green. Each segment is a quad of two triangles.
Mesh build_ring_mesh() {
    std::vector<Vertex> v;
    std::vector<GLuint> idx;
    const int seg = 64;
    for (int i = 0; i < seg; ++i) {
        double a0 = 2.0 * kPi * i / seg;
        double a1 = 2.0 * kPi * (i + 1) / seg;
        unsigned int base = static_cast<unsigned int>(v.size());
        v.push_back({(float)std::cos(a0), 0.0f, (float)std::sin(a0), 0.0f, 1.0f, 0.0f, 0.2f, 1.0f,
                     0.3f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        v.push_back({(float)(kRingInner * std::cos(a0)), 0.0f, (float)(kRingInner * std::sin(a0)),
                     0.0f, 1.0f, 0.0f, 0.2f, 1.0f, 0.3f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        v.push_back({(float)std::cos(a1), 0.0f, (float)std::sin(a1), 0.0f, 1.0f, 0.0f, 0.2f, 1.0f,
                     0.3f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        v.push_back({(float)(kRingInner * std::cos(a1)), 0.0f, (float)(kRingInner * std::sin(a1)),
                     0.0f, 1.0f, 0.0f, 0.2f, 1.0f, 0.3f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        idx.push_back(base);
        idx.push_back(base + 1);
        idx.push_back(base + 2);
        idx.push_back(base + 2);
        idx.push_back(base + 1);
        idx.push_back(base + 3);
    }
    Mesh m;
    m.upload(v, idx);
    return m;
}

}  // namespace

void Gizmo::build() {
    this->arrow_x_ = build_arrow_mesh(Vec3(1, 0, 0), 1.0f, 0.35f, 0.3f);
    this->arrow_y_ = build_arrow_mesh(Vec3(0, 1, 0), 0.35f, 1.0f, 0.4f);
    this->arrow_z_ = build_arrow_mesh(Vec3(0, 0, 1), 0.35f, 0.55f, 1.0f);
    this->ring_ = build_ring_mesh();
}

void Gizmo::destroy() {
    this->arrow_x_.destroy();
    this->arrow_y_.destroy();
    this->arrow_z_.destroy();
    this->ring_.destroy();
}

void Gizmo::render(Shader& shader, GizmoMode mode, const Vec3& center, double world_size) const {
    if (mode == GizmoMode::Off) return;
    Mat4 model = Mat4::from_position_rotation(center, Quaternion::identity()) *
                 Mat4::scale_matrix(Vec3(world_size, world_size, world_size));
    shader.set_uniform("uModel", model.ptr());
    shader.set_uniform("uAlpha", 1.0f);
    shader.set_uniform("uLightDir", 0.0f, 0.0f, 0.0f);
    shader.set_uniform("uLightColor", 0.0f, 0.0f, 0.0f);
    shader.set_uniform("uAmbientColor", 1.0f, 1.0f, 1.0f);
    shader.set_uniform("uUseModelTexture", 0);
    shader.set_uniform("uUseTerrainTexture", 0);
    if (mode == GizmoMode::Move) {
        if (this->arrow_x_.valid()) this->arrow_x_.draw();
        if (this->arrow_y_.valid()) this->arrow_y_.draw();
        if (this->arrow_z_.valid()) this->arrow_z_.draw();
    } else if (mode == GizmoMode::Rotate) {
        if (this->ring_.valid()) {
            Mat4 ring_model =
                Mat4::from_position_rotation(center, Quaternion::identity()) *
                Mat4::scale_matrix(Vec3(world_size * kRingScale, world_size * kRingScale,
                                        world_size * kRingScale));
            shader.set_uniform("uModel", ring_model.ptr());
            this->ring_.draw();
        }
    }
    shader.set_uniform("uAmbientColor", 0.4f, 0.42f, 0.45f);
    shader.set_uniform("uLightColor", 1.0f, 0.95f, 0.85f);
    shader.set_uniform("uLightDir", 0.5f, 1.0f, 0.3f);
}

GizmoHandle Gizmo::pick(const Camera& cam, int vp_x, int vp_y, int vp_w, int vp_h, double mx,
                        double my, GizmoMode mode, const Vec3& center, double world_size) const {
    if (mode == GizmoMode::Off) return GizmoHandle::None;
    const double threshold = 12.0;
    Vec2 cursor(mx - vp_x, my - vp_y);
    if (mode == GizmoMode::Move) {
        auto dist_to = [&](const Vec3& tip) {
            Vec2 a = project_to_screen(cam, vp_w, vp_h, center);
            Vec2 b = project_to_screen(cam, vp_w, vp_h, center + tip * world_size);
            return point_segment_distance(cursor, a, b);
        };
        if (dist_to(Vec3(1, 0, 0)) < threshold) return GizmoHandle::AxisX;
        if (dist_to(Vec3(0, 1, 0)) < threshold) return GizmoHandle::AxisY;
        if (dist_to(Vec3(0, 0, 1)) < threshold) return GizmoHandle::AxisZ;
    } else if (mode == GizmoMode::Rotate) {
        double ring_radius = world_size * kRingScale;
        if (robcraft::editor::pick_yaw_ring(cam, mx, my, vp_x, vp_y, vp_w, vp_h, center,
                                            ring_radius, kRingInner, threshold))
            return GizmoHandle::Yaw;
    }
    return GizmoHandle::None;
}

EditorGizmo::EditorGizmo(EditorApp& app) : app_(app) {}

void EditorGizmo::render_gizmo() {
    if (this->app_.current_tool_ != EditorTool::Select || this->app_.selection_.empty()) return;
    if (this->app_.gizmo_mode_ == GizmoMode::Off) return;
    auto* tf =
        this->app_.world_.get_component<Transform3D>(this->app_.placement_.primary_selection());
    if (!tf) return;
    double size = robcraft::editor::gizmo_world_size(this->app_.editor_camera_, tf->position,
                                                     this->app_.viewport_h_);
    this->app_.gizmo_.render(this->app_.shader_, this->app_.gizmo_mode_, tf->position, size);
}

bool EditorGizmo::handle_gizmo_press() {
    if (this->app_.current_tool_ != EditorTool::Select || this->app_.selection_.empty())
        return false;
    if (this->app_.gizmo_mode_ == GizmoMode::Off) return false;
    auto* tf =
        this->app_.world_.get_component<Transform3D>(this->app_.placement_.primary_selection());
    if (!tf) return false;

    double mx, my;
    glfwGetCursorPos(this->app_.window_, &mx, &my);
    double size = robcraft::editor::gizmo_world_size(this->app_.editor_camera_, tf->position,
                                                     this->app_.viewport_h_);
    GizmoHandle h = this->app_.gizmo_.pick(this->app_.editor_camera_, this->app_.viewport_x_,
                                           this->app_.viewport_y_, this->app_.viewport_w_,
                                           this->app_.viewport_h_, mx, my, this->app_.gizmo_mode_,
                                           tf->position, size);
    if (h == GizmoHandle::None) return false;

    this->app_.gizmo_handle_ = h;
    this->app_.gizmo_center_ = tf->position;
    this->app_.gizmo_drag_active_ = true;
    this->app_.gizmo_start_transforms_.clear();
    for (Entity e : this->app_.selection_) {
        auto* t = this->app_.world_.get_component<Transform3D>(e);
        if (t) this->app_.gizmo_start_transforms_.emplace_back(e, *t);
    }
    this->app_.undo_recorder_.begin();
    for (Entity e : this->app_.selection_) {
        if (this->app_.world_.get_component<Transform3D>(e))
            this->app_.undo_recorder_.record_before_entity(e);
    }
    Ray ray = this->app_.placement_.cursor_ray();
    Vec3 n = (h == GizmoHandle::Yaw) ? Vec3(0, 1, 0) : this->app_.editor_camera_.forward();
    auto t = robcraft::editor::ray_plane_intersect(ray, this->app_.gizmo_center_, n);
    this->app_.gizmo_grab_pt_ = t ? (ray.origin + ray.direction * *t) : this->app_.gizmo_center_;
    if (h == GizmoHandle::Yaw) {
        this->app_.gizmo_last_angle_ =
            robcraft::editor::yaw_angle(this->app_.gizmo_center_, this->app_.gizmo_grab_pt_);
        this->app_.gizmo_drag_angle_ = 0.0;
    }
    return true;
}

void EditorGizmo::update_gizmo_drag() {
    if (!this->app_.gizmo_drag_active_) return;
    Ray ray = this->app_.placement_.cursor_ray();

    if (this->app_.gizmo_handle_ == GizmoHandle::Yaw) {
        auto t =
            robcraft::editor::ray_plane_intersect(ray, this->app_.gizmo_center_, Vec3(0, 1, 0));
        if (!t) return;
        Vec3 p = ray.origin + ray.direction * *t;
        double cur = robcraft::editor::yaw_angle(this->app_.gizmo_center_, p);
        // Accumulate the shortest-path increment so a continuous drag never
        // jumps when atan2 wraps from +180 to -180 degrees.
        this->app_.gizmo_drag_angle_ +=
            robcraft::editor::shortest_angle_delta(this->app_.gizmo_last_angle_, cur);
        this->app_.gizmo_last_angle_ = cur;
        double delta = robcraft::editor::snap_angle(this->app_.gizmo_drag_angle_);
        Quaternion yaw = Quaternion::from_axis_angle(Vec3(0, 1, 0), delta);
        for (auto& [e, start] : this->app_.gizmo_start_transforms_) {
            if (auto* tf = this->app_.world_.get_component<Transform3D>(e))
                tf->rotation = yaw * start.rotation;
        }
        return;
    }

    auto t = robcraft::editor::ray_plane_intersect(ray, this->app_.gizmo_grab_pt_,
                                                   this->app_.editor_camera_.forward());
    if (!t) return;
    Vec3 p = ray.origin + ray.direction * *t;
    Vec3 axis;
    if (this->app_.gizmo_handle_ == GizmoHandle::AxisX)
        axis = Vec3(1, 0, 0);
    else if (this->app_.gizmo_handle_ == GizmoHandle::AxisY)
        axis = Vec3(0, 1, 0);
    else
        axis = Vec3(0, 0, 1);
    double delta = (p - this->app_.gizmo_grab_pt_).dot(axis);
    bool horizontal = this->app_.gizmo_handle_ != GizmoHandle::AxisY;
    bool has_terrain = this->app_.world_.has_terrain();
    for (auto& [e, start] : this->app_.gizmo_start_transforms_) {
        auto* tf = this->app_.world_.get_component<Transform3D>(e);
        if (!tf) continue;
        Vec3 pos = start.position;
        if (this->app_.gizmo_handle_ == GizmoHandle::AxisX)
            pos.x = start.position.x + delta;
        else if (this->app_.gizmo_handle_ == GizmoHandle::AxisZ)
            pos.z = start.position.z + delta;
        else
            pos.y = start.position.y + delta;
        if (horizontal && has_terrain) {
            double offset = start.position.y - this->app_.world_.terrain().height_at_world(
                                                   start.position.x, start.position.z);
            pos.y = this->app_.world_.terrain().height_at_world(pos.x, pos.z) + offset;
        }
        tf->position = pos;
    }
}

void EditorGizmo::end_gizmo_drag(bool cancel) {
    if (!this->app_.gizmo_drag_active_) return;
    if (cancel) {
        for (auto& [e, start] : this->app_.gizmo_start_transforms_) {
            if (auto* tf = this->app_.world_.get_component<Transform3D>(e)) *tf = start;
        }
        this->app_.undo_recorder_.begin();
    } else {
        // Skip a no-op drag (handle grabbed but not moved).
        bool changed = false;
        bool scale_changed = false;
        for (auto& [e, start] : this->app_.gizmo_start_transforms_) {
            auto* tf = this->app_.world_.get_component<Transform3D>(e);
            if (!tf) continue;
            if (tf->scale != start.scale) scale_changed = true;
            if (tf->position != start.position ||
                tf->rotation.to_euler() != start.rotation.to_euler() || tf->scale != start.scale) {
                changed = true;
                break;
            }
        }
        if (changed) {
            for (auto& [e, start] : this->app_.gizmo_start_transforms_) {
                if (this->app_.world_.get_component<Transform3D>(e)) {
                    this->app_.undo_recorder_.record_after_entity(e);
                }
            }
            this->app_.tools_.mark_modified();
            this->app_.commit_undo("Transform");
            // A scale drag resizes the rendered model; refit colliders so the
            // selection box and physics keep hugging the model.
            if (scale_changed) {
                robcraft::renderer::refit_world_colliders(this->app_.world_,
                                                          this->app_.model_cache_);
            }
            // The selected entity changed outside an inspector edit session;
            // drop the stale baseline so the next inspector edit is correct.
            this->app_.inspector_before_.reset();
        } else {
            this->app_.undo_recorder_.begin();
        }
    }
    this->app_.gizmo_drag_active_ = false;
    this->app_.gizmo_handle_ = GizmoHandle::None;
    this->app_.gizmo_start_transforms_.clear();
}

void EditorGizmo::cycle_gizmo_mode() {
    switch (this->app_.gizmo_mode_) {
        case GizmoMode::Off:
            this->app_.gizmo_mode_ = GizmoMode::Move;
            break;
        case GizmoMode::Move:
            this->app_.gizmo_mode_ = GizmoMode::Rotate;
            break;
        case GizmoMode::Rotate:
            this->app_.gizmo_mode_ = GizmoMode::Off;
            break;
    }
}

}  // namespace robcraft::editor
