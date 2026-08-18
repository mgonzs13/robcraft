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

#include <optional>

#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/vec2.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/renderer/camera.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::math;
using namespace robcraft::renderer;

/**
 * @brief Projects a world point to viewport pixel coordinates, relative to the
 * viewport's top-left corner (Y downward, matching cursor coords).
 * @param cam The camera.
 * @param vp_w Viewport width in pixels.
 * @param vp_h Viewport height in pixels.
 * @param world The world-space point.
 * @return Viewport-relative pixel position. Points behind the camera are not
 *         culled and may project to mirrored coordinates.
 */
Vec2 project_to_screen(const Camera& cam, int vp_w, int vp_h, const Vec3& world);

/**
 * @brief Distance from a point to a line segment in 2D.
 * @param p The point.
 * @param a Segment start.
 * @param b Segment end.
 * @return Closest distance to the segment.
 */
double point_segment_distance(const Vec2& p, const Vec2& a, const Vec2& b);

/**
 * @brief Intersects a ray with an infinite plane.
 * @param ray The ray.
 * @param plane_pt A point on the plane.
 * @param plane_normal Plane normal (need not be normalized).
 * @return Ray parameter t at the hit, or nullopt if parallel or behind the origin.
 */
std::optional<double> ray_plane_intersect(const Ray& ray, const Vec3& plane_pt,
                                          const Vec3& plane_normal);

/**
 * @brief Yaw angle (radians) from a center toward a point in the XZ plane.
 * @param center Reference point.
 * @param point Target point.
 * @return atan2(dx, dz) in radians.
 */
double yaw_angle(const Vec3& center, const Vec3& point);

/**
 * @brief Rounds an angle to the nearest multiple of a fixed step.
 * @param angle_rad Angle in radians.
 * @param step_deg Snap step in degrees (default 15).
 * @return Snapped angle in radians.
 */
double snap_angle(double angle_rad, double step_deg = 15.0);

/**
 * @brief Smallest signed rotation (radians) from one angle to another.
 * @param a Start angle in radians.
 * @param b End angle in radians.
 * @return The shortest delta in [-pi, pi], so a continuous rotation tracked
 *         with these deltas never jumps when atan2 wraps at the +/-180 deg seam.
 */
double shortest_angle_delta(double a, double b);
/**
 * @brief World-space gizmo size for a roughly screen-constant pixel size.
 * @param cam The camera.
 * @param center Gizmo center.
 * @param vp_h Viewport height in pixels.
 * @param target_px Desired arrow length in pixels (default 140).
 * @return World-space length for the gizmo.
 */
double gizmo_world_size(const Camera& cam, const Vec3& center, int vp_h, double target_px = 140.0);

/**
 * @brief Hit-tests the cursor ray against a horizontal yaw ring (annulus).
 * @param cam The camera.
 * @param cursor_x Cursor X in window coordinates.
 * @param cursor_y Cursor Y in window coordinates.
 * @param vp_x Viewport left edge in window coordinates.
 * @param vp_y Viewport top edge in window coordinates.
 * @param vp_w Viewport width in pixels.
 * @param vp_h Viewport height in pixels.
 * @param center World position of the ring center.
 * @param outer_radius Outer ring radius in world units.
 * @param inner_frac Inner radius as a fraction of the outer radius (0..1).
 * @param tol_px Ring band tolerance in pixels.
 * @return True if the cursor ray falls on the ring's annulus.
 */
bool pick_yaw_ring(const Camera& cam, double cursor_x, double cursor_y, int vp_x, int vp_y,
                   int vp_w, int vp_h, const Vec3& center, double outer_radius, double inner_frac,
                   double tol_px);

}  // namespace robcraft::editor
