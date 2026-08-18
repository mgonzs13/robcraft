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

#include <array>

#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

/** @brief Column-major 4x4 matrix of single-precision floats. */
struct Mat4 {
    /** @brief Matrix elements stored column-major. */
    std::array<float, 16> data;

    Mat4();

    /** @brief Fills the matrix with the identity. */
    void set_identity();

    /**
     * @brief Returns a pointer to the raw element data.
     * @return Non-const pointer to the data.
     */
    float* ptr() { return this->data.data(); }

    /**
     * @brief Returns a pointer to the raw element data.
     * @return Const pointer to the data.
     */
    const float* ptr() const { return this->data.data(); }

    /**
     * @brief Matrix-matrix multiplication.
     * @param o The right-hand matrix.
     * @return The product matrix.
     */
    Mat4 operator*(const Mat4& o) const;

    /**
     * @brief Builds a perspective projection matrix.
     * @param fov_y_rad Vertical field of view in radians.
     * @param aspect Aspect ratio (width / height).
     * @param near_plane Near clip distance.
     * @param far_plane Far clip distance.
     * @return The projection matrix.
     */
    static Mat4 perspective(float fov_y_rad, float aspect, float near_plane, float far_plane);

    /**
     * @brief Builds an orthographic projection matrix (classic GL mapping).
     * @param left Left plane.
     * @param right Right plane.
     * @param bottom Bottom plane.
     * @param top Top plane.
     * @param near_plane Near plane.
     * @param far_plane Far plane.
     * @return The orthographic projection matrix.
     */
    static Mat4 orthographic(float left, float right, float bottom, float top, float near_plane,
                             float far_plane);

    /**
     * @brief Builds a view matrix looking from eye toward center.
     * @param eye Camera position.
     * @param center Target position.
     * @param up Up vector.
     * @return The view matrix.
     */
    static Mat4 look_at(const Vec3& eye, const Vec3& center, const Vec3& up);

    /**
     * @brief Builds a world matrix from a position and rotation.
     * @param pos Translation.
     * @param q Rotation.
     * @return The combined transform matrix.
     */
    static Mat4 from_position_rotation(const Vec3& pos, const Quaternion& q);

    /**
     * @brief Builds a non-uniform scale matrix.
     * @param s Scale factors per axis.
     * @return The scale matrix.
     */
    static Mat4 scale_matrix(const Vec3& s);

    /**
     * @brief Returns the inverse of this matrix.
     * @return The inverse, or the identity matrix when this matrix is singular.
     */
    Mat4 inverse() const;
};

}  // namespace robcraft::engine::math
