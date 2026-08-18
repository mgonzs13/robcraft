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
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::renderer {

class Camera;

using namespace robcraft::engine::collision;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

/**
 * @brief Build a cursor ray in world space from a camera and viewport coordinates.
 * @param cam The camera.
 * @param cursor_x Cursor x (window coords).
 * @param cursor_y Cursor y (window coords).
 * @param vp_x Viewport origin x (window coords).
 * @param vp_y Viewport origin y (window coords).
 * @param vp_w Viewport width in pixels.
 * @param vp_h Viewport height in pixels.
 * @return The cursor ray. */
Ray cursor_ray(const Camera& cam, double cursor_x, double cursor_y, int vp_x, int vp_y, int vp_w,
               int vp_h);

/**
 * @brief Returns the world point under a cursor position.
 * @param cam The camera used to build the pick ray.
 * @param world The world to pick against (colliders first, then terrain).
 * @param cursor_x Cursor X in window coordinates.
 * @param cursor_y Cursor Y in window coordinates.
 * @param vp_x Viewport left edge in window coordinates.
 * @param vp_y Viewport top edge in window coordinates.
 * @param vp_w Viewport width in pixels.
 * @param vp_h Viewport height in pixels.
 * @return The closest object hit point, else the terrain hit point, else nullopt.
 */
std::optional<Vec3> pick_world_point(const Camera& cam, const World& world, double cursor_x,
                                     double cursor_y, int vp_x, int vp_y, int vp_w, int vp_h);

/**
 * @brief Returns where the cursor ray crosses a horizontal plane.
 * @param cam The camera used to build the cursor ray.
 * @param cursor_x Cursor X in window coordinates.
 * @param cursor_y Cursor Y in window coordinates.
 * @param vp_x Viewport left edge in window coordinates.
 * @param vp_y Viewport top edge in window coordinates.
 * @param vp_w Viewport width in pixels.
 * @param vp_h Viewport height in pixels.
 * @param plane_y World Y of the horizontal plane.
 * @return The intersection point, or nullopt if the ray is parallel or points away.
 */
std::optional<Vec3> pick_point_on_plane(const Camera& cam, double cursor_x, double cursor_y,
                                        int vp_x, int vp_y, int vp_w, int vp_h, double plane_y);

}  // namespace robcraft::renderer
