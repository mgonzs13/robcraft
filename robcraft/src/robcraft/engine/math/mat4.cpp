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

#include "robcraft/engine/math/mat4.hpp"

#include <algorithm>
#include <cmath>

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

Mat4::Mat4() {
    this->set_identity();
}

void Mat4::set_identity() {
    this->data.fill(0.0f);
    this->data[0] = this->data[5] = this->data[10] = this->data[15] = 1.0f;
}

Mat4 Mat4::operator*(const Mat4& o) const {
    Mat4 r;
    r.data.fill(0.0f);
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            for (int k = 0; k < 4; ++k) {
                r.data[col * 4 + row] += this->data[k * 4 + row] * o.data[col * 4 + k];
            }
        }
    }
    return r;
}

Mat4 Mat4::perspective(float fov_y_rad, float aspect, float near_plane, float far_plane) {
    Mat4 m;
    float tan_half = std::tan(fov_y_rad / 2.0f);
    m.data.fill(0.0f);
    m.data[0] = 1.0f / (aspect * tan_half);
    m.data[5] = 1.0f / tan_half;
    m.data[10] = -(far_plane + near_plane) / (far_plane - near_plane);
    m.data[11] = -1.0f;
    m.data[14] = -(2.0f * far_plane * near_plane) / (far_plane - near_plane);
    return m;
}

Mat4 Mat4::orthographic(float left, float right, float bottom, float top, float near_plane,
                        float far_plane) {
    Mat4 m;
    m.data.fill(0.0f);
    m.data[0] = 2.0f / (right - left);
    m.data[5] = 2.0f / (top - bottom);
    m.data[10] = -2.0f / (far_plane - near_plane);
    m.data[12] = -(right + left) / (right - left);
    m.data[13] = -(top + bottom) / (top - bottom);
    m.data[14] = -(far_plane + near_plane) / (far_plane - near_plane);
    m.data[15] = 1.0f;
    return m;
}

Mat4 Mat4::look_at(const Vec3& eye, const Vec3& center, const Vec3& up) {
    Vec3 f = (center - eye).normalized();
    Vec3 s = f.cross(up).normalized();
    Vec3 u = s.cross(f);

    Mat4 m;
    m.data[0] = static_cast<float>(s.x);
    m.data[4] = static_cast<float>(s.y);
    m.data[8] = static_cast<float>(s.z);
    m.data[12] = -static_cast<float>(s.dot(eye));

    m.data[1] = static_cast<float>(u.x);
    m.data[5] = static_cast<float>(u.y);
    m.data[9] = static_cast<float>(u.z);
    m.data[13] = -static_cast<float>(u.dot(eye));

    m.data[2] = -static_cast<float>(f.x);
    m.data[6] = -static_cast<float>(f.y);
    m.data[10] = -static_cast<float>(f.z);
    m.data[14] = static_cast<float>(f.dot(eye));

    m.data[3] = 0.0f;
    m.data[7] = 0.0f;
    m.data[11] = 0.0f;
    m.data[15] = 1.0f;

    return m;
}

Mat4 Mat4::from_position_rotation(const Vec3& pos, const Quaternion& q) {
    double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    double xx = q.x * q.x, xy = q.x * q.y, xz = q.x * q.z;
    double yy = q.y * q.y, yz = q.y * q.z, zz = q.z * q.z;

    Mat4 m;
    m.data[0] = static_cast<float>(1.0 - 2.0 * (yy + zz));
    m.data[4] = static_cast<float>(2.0 * (xy - wz));
    m.data[8] = static_cast<float>(2.0 * (xz + wy));
    m.data[12] = static_cast<float>(pos.x);

    m.data[1] = static_cast<float>(2.0 * (xy + wz));
    m.data[5] = static_cast<float>(1.0 - 2.0 * (xx + zz));
    m.data[9] = static_cast<float>(2.0 * (yz - wx));
    m.data[13] = static_cast<float>(pos.y);

    m.data[2] = static_cast<float>(2.0 * (xz - wy));
    m.data[6] = static_cast<float>(2.0 * (yz + wx));
    m.data[10] = static_cast<float>(1.0 - 2.0 * (xx + yy));
    m.data[14] = static_cast<float>(pos.z);

    m.data[3] = 0.0f;
    m.data[7] = 0.0f;
    m.data[11] = 0.0f;
    m.data[15] = 1.0f;

    return m;
}

Mat4 Mat4::scale_matrix(const Vec3& s) {
    Mat4 m;
    m.data.fill(0.0f);
    m.data[0] = static_cast<float>(s.x);
    m.data[5] = static_cast<float>(s.y);
    m.data[10] = static_cast<float>(s.z);
    m.data[15] = 1.0f;
    return m;
}

Mat4 Mat4::inverse() const {
    Mat4 m = *this;
    Mat4 inv;
    inv.set_identity();
    for (int col = 0; col < 4; ++col) {
        int pivot_row = col;
        for (int r = col + 1; r < 4; ++r) {
            if (std::abs(m.data[col * 4 + r]) > std::abs(m.data[col * 4 + pivot_row]))
                pivot_row = r;
        }
        if (std::abs(m.data[col * 4 + pivot_row]) < 1e-9f) {
            inv.set_identity();
            return inv;
        }
        if (pivot_row != col) {
            for (int c = 0; c < 4; ++c) {
                std::swap(m.data[c * 4 + col], m.data[c * 4 + pivot_row]);
                std::swap(inv.data[c * 4 + col], inv.data[c * 4 + pivot_row]);
            }
        }
        float pivot = m.data[col * 4 + col];
        for (int c = 0; c < 4; ++c) {
            m.data[c * 4 + col] /= pivot;
            inv.data[c * 4 + col] /= pivot;
        }
        for (int r = 0; r < 4; ++r) {
            if (r == col) continue;
            float factor = m.data[col * 4 + r];
            for (int c = 0; c < 4; ++c) {
                m.data[c * 4 + r] -= factor * m.data[c * 4 + col];
                inv.data[c * 4 + r] -= factor * inv.data[c * 4 + col];
            }
        }
    }
    return inv;
}

}  // namespace robcraft::engine::math
