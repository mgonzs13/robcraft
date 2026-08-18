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

#include "robcraft/editor/gizmo_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "robcraft/engine/math/constants.hpp"
#include "robcraft/renderer/pick.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::math;
using namespace robcraft::renderer;

Vec2 project_to_screen(const Camera& cam, int vp_w, int vp_h, const Vec3& world) {
    Mat4 vp = cam.projection_matrix() * cam.view_matrix();
    const std::array<float, 16>& d = vp.data;
    double x = world.x, y = world.y, z = world.z, w = 1.0;
    double cx = d[0] * x + d[4] * y + d[8] * z + d[12] * w;
    double cy = d[1] * x + d[5] * y + d[9] * z + d[13] * w;
    double cz = d[2] * x + d[6] * y + d[10] * z + d[14] * w;
    double cw = d[3] * x + d[7] * y + d[11] * z + d[15] * w;
    double ndc_x = (std::abs(cw) > 1e-12) ? cx / cw : 0.0;
    double ndc_y = (std::abs(cw) > 1e-12) ? cy / cw : 0.0;
    return Vec2((ndc_x * 0.5 + 0.5) * vp_w, (1.0 - (ndc_y * 0.5 + 0.5)) * vp_h);
}

double point_segment_distance(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    double len2 = ab.x * ab.x + ab.y * ab.y;
    if (len2 < 1e-12) {
        Vec2 d = p - a;
        return std::sqrt(d.x * d.x + d.y * d.y);
    }
    double t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2;
    t = std::clamp(t, 0.0, 1.0);
    Vec2 q(a.x + t * ab.x, a.y + t * ab.y);
    Vec2 d = p - q;
    return std::sqrt(d.x * d.x + d.y * d.y);
}

std::optional<double> ray_plane_intersect(const Ray& ray, const Vec3& plane_pt,
                                          const Vec3& plane_normal) {
    double denom = ray.direction.dot(plane_normal);
    if (std::abs(denom) < 1e-9) return std::nullopt;
    double t = (plane_pt - ray.origin).dot(plane_normal) / denom;
    if (t < 0.0) return std::nullopt;
    return t;
}

double yaw_angle(const Vec3& center, const Vec3& point) {
    return std::atan2(point.x - center.x, point.z - center.z);
}

double snap_angle(double angle_rad, double step_deg) {
    if (step_deg <= 0.0) return angle_rad;
    double step = robcraft::engine::math::deg_to_rad(step_deg);
    return std::round(angle_rad / step) * step;
}

double shortest_angle_delta(double a, double b) {
    return std::atan2(std::sin(b - a), std::cos(b - a));
}

bool pick_yaw_ring(const Camera& cam, double cursor_x, double cursor_y, int vp_x, int vp_y,
                   int vp_w, int vp_h, const Vec3& center, double outer_radius, double inner_frac,
                   double tol_px) {
    if (inner_frac <= 0.0 || inner_frac >= 1.0) return false;
    Vec2 c = project_to_screen(cam, vp_w, vp_h, center);
    Vec2 p_out = project_to_screen(cam, vp_w, vp_h, center + Vec3(outer_radius, 0, 0));
    double r_out = (p_out - c).length();
    double world_per_px = (r_out > 1e-6) ? outer_radius / r_out : 0.0;
    double tol_w = tol_px * world_per_px;

    auto hit = pick_point_on_plane(cam, cursor_x, cursor_y, vp_x, vp_y, vp_w, vp_h, center.y);
    if (!hit) return false;
    double dx = hit->x - center.x;
    double dz = hit->z - center.z;
    double r = std::sqrt(dx * dx + dz * dz);
    double r_in = outer_radius * inner_frac;
    return r >= r_in - tol_w && r <= outer_radius + tol_w;
}

double gizmo_world_size(const Camera& cam, const Vec3& center, int vp_h, double target_px) {
    double dist = (center - cam.position()).length();
    double half_fov = robcraft::engine::math::deg_to_rad(cam.fov()) * 0.5;
    return 2.0 * dist * std::tan(half_fov) * target_px / vp_h;
}

}  // namespace robcraft::editor
