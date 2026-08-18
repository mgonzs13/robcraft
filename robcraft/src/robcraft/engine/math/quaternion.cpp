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

#include "robcraft/engine/math/quaternion.hpp"

#include <cmath>

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

Quaternion Quaternion::identity() {
    return {1.0, 0.0, 0.0, 0.0};
}

Quaternion Quaternion::from_axis_angle(const Vec3& axis, double angle_rad) {
    double half = angle_rad * 0.5;
    double s = std::sin(half);
    Vec3 n = axis.normalized();
    return {std::cos(half), n.x * s, n.y * s, n.z * s};
}

Quaternion Quaternion::from_euler(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return {this->w * q.w - this->x * q.x - this->y * q.y - this->z * q.z,
            this->w * q.x + this->x * q.w + this->y * q.z - this->z * q.y,
            this->w * q.y - this->x * q.z + this->y * q.w + this->z * q.x,
            this->w * q.z + this->x * q.y - this->y * q.x + this->z * q.w};
}

Vec3 Quaternion::rotate(const Vec3& v) const {
    Vec3 qv{this->x, this->y, this->z};
    Vec3 uv = qv.cross(v);
    Vec3 uuv = qv.cross(uv);
    return v + (uv * this->w + uuv) * 2.0;
}

Quaternion Quaternion::conjugate() const {
    return {this->w, -this->x, -this->y, -this->z};
}

Quaternion Quaternion::normalized() const {
    double len =
        std::sqrt(this->w * this->w + this->x * this->x + this->y * this->y + this->z * this->z);
    return len > 0.0 ? Quaternion{this->w / len, this->x / len, this->y / len, this->z / len}
                     : identity();
}

Vec3 Quaternion::to_euler() const {
    Quaternion q = this->normalized();
    // roll (about X)
    double sr_cp = 2.0 * (q.w * q.x + q.y * q.z);
    double cr_cp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sr_cp, cr_cp);

    // pitch (about Y)
    double sp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch;
    if (sp > 1.0) sp = 1.0;
    if (sp < -1.0) sp = -1.0;
    pitch = std::asin(sp);

    // yaw (about Z)
    double sr_cp2 = 2.0 * (q.w * q.z + q.x * q.y);
    double cr_cp2 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(sr_cp2, cr_cp2);

    return Vec3(roll, pitch, yaw);
}

}  // namespace robcraft::engine::math
