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

#include "robcraft/renderer/pick.hpp"

#include <cmath>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/camera.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

/** @brief Build a cursor ray in world space from a camera and viewport coordinates. */
Ray cursor_ray(const Camera& cam, double cursor_x, double cursor_y, int vp_x, int vp_y, int vp_w,
               int vp_h) {
    float ndc_x = 2.0f * static_cast<float>(cursor_x - vp_x) / vp_w - 1.0f;
    float ndc_y = 1.0f - 2.0f * static_cast<float>(cursor_y - vp_y) / vp_h;

    Vec3 ro = cam.position();
    Vec3 rd = cam.forward();
    Vec3 right = rd.cross(Vec3(0.0, 1.0, 0.0)).normalized();
    if (right.length_sq() < 1e-12) right = Vec3(1.0, 0.0, 0.0);
    Vec3 up = right.cross(rd).normalized();
    float th = std::tan(static_cast<float>(robcraft::engine::math::deg_to_rad(cam.fov())) * 0.5f);
    float asp = static_cast<float>(vp_w) / vp_h;
    Vec3 dir = (rd + right * (ndc_x * asp * th) + up * (ndc_y * th)).normalized();
    return Ray{ro, dir};
}

std::optional<Vec3> pick_world_point(const Camera& cam, const World& world, double cursor_x,
                                     double cursor_y, int vp_x, int vp_y, int vp_w, int vp_h) {
    Ray ray = cursor_ray(cam, cursor_x, cursor_y, vp_x, vp_y, vp_w, vp_h);
    Vec3 ro = ray.origin;
    Vec3 dir = ray.direction;

    auto* cs = world.store<BoxCollider>();
    auto* ts = world.store<Transform3D>();
    if (cs && ts) {
        std::optional<double> closest;
        for (auto& [e, col] : *cs) {
            auto* tf = ts->get(e);
            if (!tf) continue;
            auto aabb = AABB::from_box(tf->position, col, tf->rotation);
            auto hit = ray_aabb_intersection(ray, aabb);
            if (hit.has_value() && (!closest.has_value() || *hit < *closest)) closest = hit;
        }
        if (closest.has_value()) return {ro + dir * *closest};
    }

    if (world.has_terrain()) {
        auto t = world.terrain().raycast(ray);
        if (t.has_value()) return {ro + dir * *t};
    }

    return std::nullopt;
}

std::optional<Vec3> pick_point_on_plane(const Camera& cam, double cursor_x, double cursor_y,
                                        int vp_x, int vp_y, int vp_w, int vp_h, double plane_y) {
    Ray ray = cursor_ray(cam, cursor_x, cursor_y, vp_x, vp_y, vp_w, vp_h);
    if (std::abs(ray.direction.y) < 1e-12) return std::nullopt;
    double t = (plane_y - ray.origin.y) / ray.direction.y;
    if (t < 0.0) return std::nullopt;
    return {ray.origin + ray.direction * t};
}

}  // namespace robcraft::renderer
