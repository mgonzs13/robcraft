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

#include "robcraft/renderer/camera.hpp"

#include <algorithm>
#include <cmath>

#include "robcraft/engine/math/constants.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

void Camera::set_perspective(float fov_deg, float aspect, float near_plane, float far_plane) {
    this->fov_ = fov_deg;
    this->aspect_ = aspect;
    this->near_ = near_plane;
    this->far_ = far_plane;
    this->update_projection();
}

void Camera::set_position(const Vec3& pos) {
    this->position_ = pos;
}

void Camera::look_at(const Vec3& target) {
    this->forward_ = (target - this->position_).normalized();
    this->rebuild_basis();

    this->yaw_ = std::atan2(this->forward_.x, this->forward_.z);
    this->pitch_ = std::asin(std::clamp(this->forward_.y, -1.0, 1.0));
}

void Camera::move_forward(float amount) {
    this->position_ = this->position_ + this->forward_ * amount;
}

void Camera::move_world_up(float amount) {
    Vec3 delta(0.0, amount, 0.0);
    this->position_ = this->position_ + delta;
    if (this->has_orbit_target_) this->orbit_target_ = this->orbit_target_ + delta;
}

void Camera::rotate(float yaw_delta, float pitch_delta) {
    this->yaw_ += yaw_delta;
    this->pitch_ += pitch_delta;
    if (this->pitch_ > 1.5f) this->pitch_ = 1.5f;
    if (this->pitch_ < -1.5f) this->pitch_ = -1.5f;

    this->forward_.x = std::cos(this->pitch_) * std::sin(this->yaw_);
    this->forward_.y = std::sin(this->pitch_);
    this->forward_.z = std::cos(this->pitch_) * std::cos(this->yaw_);
    this->rebuild_basis();
}

void Camera::orbit(float yaw_delta, float pitch_delta, float distance) {
    if (!this->has_orbit_target_) return;
    this->yaw_ += yaw_delta;
    this->pitch_ += pitch_delta;
    if (this->pitch_ > 1.5f) this->pitch_ = 1.5f;
    if (this->pitch_ < -1.5f) this->pitch_ = -1.5f;

    this->forward_.x = std::cos(this->pitch_) * std::sin(this->yaw_);
    this->forward_.y = std::sin(this->pitch_);
    this->forward_.z = std::cos(this->pitch_) * std::cos(this->yaw_);

    this->position_ = this->orbit_target_ - this->forward_ * distance;
    this->rebuild_basis();
}

void Camera::set_orbit_target(const Vec3& target) {
    this->orbit_target_ = target;
    this->has_orbit_target_ = true;
}

void Camera::zoom_by_factor(float factor) {
    if (!this->has_orbit_target_) return;
    Vec3 offset = this->orbit_target_ - this->position_;
    double dist = offset.length();
    if (dist < 1e-6) return;
    double new_dist =
        std::clamp(dist * static_cast<double>(factor), static_cast<double>(this->min_orbit_dist_),
                   static_cast<double>(this->max_orbit_dist_));
    this->position_ = this->orbit_target_ - offset.normalized() * new_dist;
}

void Camera::pan(float forward_amount, float right_amount) {
    Vec3 fwd = this->forward_;
    fwd.y = 0.0;
    if (fwd.length_sq() > 1e-9) fwd = fwd.normalized();
    Vec3 delta = fwd * forward_amount + this->right_ * right_amount;
    this->position_ = this->position_ + delta;
    if (this->has_orbit_target_) this->orbit_target_ = this->orbit_target_ + delta;
}

Mat4 Camera::view_matrix() const {
    return Mat4::look_at(this->position_, this->position_ + this->forward_, this->up_);
}

void Camera::update_projection() {
    float fov_rad = static_cast<float>(robcraft::engine::math::deg_to_rad(this->fov_));
    this->projection_ = Mat4::perspective(fov_rad, this->aspect_, this->near_, this->far_);
}

void Camera::rebuild_basis() {
    this->right_ = this->forward_.cross(this->world_up_).normalized();
    this->up_ = this->right_.cross(this->forward_).normalized();
}

}  // namespace robcraft::renderer
