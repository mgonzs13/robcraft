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

#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

/** @brief Perspective camera supporting free movement and orbit controls. */
class Camera {
public:
    Camera() = default;

    /** @brief Set the perspective projection parameters.
     *  @param fov_deg Vertical field of view in degrees.
     *  @param aspect Aspect ratio (width / height).
     *  @param near_plane Near clip plane distance.
     *  @param far_plane Far clip plane distance. */
    void set_perspective(float fov_deg, float aspect, float near_plane, float far_plane);

    /** @brief Set the camera position.
     *  @param pos New world-space position. */
    void set_position(const Vec3& pos);

    /** @brief Orient the camera to look at a target.
     *  @param target World-space point to look at. */
    void look_at(const Vec3& target);

    /** @brief Move the camera along its forward axis.
     *  @param amount Signed distance to move. */
    void move_forward(float amount);

    /** @brief Move the camera straight along the world Y axis.
     *  @param amount Signed distance to move.
     *  The orbit target moves with the camera, so orbiting stays stable. */
    void move_world_up(float amount);

    /** @brief Rotate the camera view in place (free-look).
     *  @param yaw_delta Yaw change in radians.
     *  @param pitch_delta Pitch change in radians (clamped to +/- 1.5 rad). */
    void rotate(float yaw_delta, float pitch_delta);

    /** @brief Orbit the camera around its target.
     *  @param yaw_delta Yaw change in radians.
     *  @param pitch_delta Pitch change in radians (clamped).
     *  @param distance Distance to keep from the target. */
    void orbit(float yaw_delta, float pitch_delta, float distance);

    /** @brief Zoom by scaling the distance to the orbit target.
     *  @param factor Scale factor (>1 zooms out, <1 zooms in), clamped to min/max. */
    void zoom_by_factor(float factor);

    /** @brief Move the camera in the horizontal plane (pan).
     *  @param forward_amount Signed distance along the flattened forward axis.
     *  @param right_amount Signed distance along the right axis.
     *  The orbit target moves with the camera. */
    void pan(float forward_amount, float right_amount);

    /** @brief Set the orbit target point.
     *  @param target World-space orbit center. */
    void set_orbit_target(const Vec3& target);

    /** @return Current orbit target point. */
    Vec3 orbit_target() const { return this->orbit_target_; }

    /** @return True once an orbit target has been assigned. */
    bool has_orbit_target() const { return this->has_orbit_target_; }

    /** @return The view matrix for this camera. */
    Mat4 view_matrix() const;

    /** @return The projection matrix for this camera. */
    Mat4 projection_matrix() const { return this->projection_; }

    /** @return Current camera position. */
    Vec3 position() const { return this->position_; }

    /** @return Current forward direction. */
    Vec3 forward() const { return this->forward_; }

    /** @return Vertical field of view in degrees. */
    float fov() const { return this->fov_; }

    /** @return Current aspect ratio. */
    float aspect() const { return this->aspect_; }

    /** @return Near clip plane distance. */
    float near_plane() const { return this->near_; }

    /** @return Far clip plane distance. */
    float far_plane() const { return this->far_; }

private:
    /** @brief Recompute the projection matrix from the current parameters. */
    void update_projection();

    /** @brief Recompute the right and up basis vectors from the current forward vector.
     *  @note Keeps the basis orthonormal; the forward vector must be set first. */
    void rebuild_basis();

    /** @brief Camera position. */
    Vec3 position_{0.0, 5.0, 10.0};
    /** @brief Forward view direction. */
    Vec3 forward_{0.0, -0.3, -0.95};
    /** @brief Up vector. */
    Vec3 up_{0.0, 1.0, 0.0};
    /** @brief Right vector. */
    Vec3 right_{1.0, 0.0, 0.0};
    /** @brief Global up reference used by look-at and orbit. */
    Vec3 world_up_{0.0, 1.0, 0.0};
    /** @brief Orbit pivot point. */
    Vec3 orbit_target_{0.0, 0.0, 0.0};
    /** @brief Whether an orbit pivot has been assigned. */
    bool has_orbit_target_ = false;
    /** @brief Minimum camera distance from the orbit target. */
    float min_orbit_dist_ = 0.5f;
    /** @brief Maximum camera distance from the orbit target. */
    float max_orbit_dist_ = 500.0f;

    /** @brief Vertical field of view in degrees. */
    float fov_ = 60.0f;
    /** @brief Aspect ratio (width / height). */
    float aspect_ = 4.0f / 3.0f;
    /** @brief Near clip plane distance. */
    float near_ = 0.1f;
    /** @brief Far clip plane distance. */
    float far_ = 500.0f;

    /** @brief Current yaw angle in radians. */
    float yaw_ = -1.0f;
    /** @brief Current pitch angle in radians. */
    float pitch_ = -0.3f;

    /** @brief Cached projection matrix. */
    Mat4 projection_;
};

}  // namespace robcraft::renderer
