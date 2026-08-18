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

#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

/** @brief Unit quaternion representing a rotation. */
struct Quaternion {
    /** @brief Real (scalar) part. */
    double w = 1.0;
    /** @brief Imaginary X part. */
    double x = 0.0;
    /** @brief Imaginary Y part. */
    double y = 0.0;
    /** @brief Imaginary Z part. */
    double z = 0.0;

    Quaternion() = default;

    /**
     * @brief Constructs a quaternion from explicit components.
     * @param w Real part.
     * @param x Imaginary X part.
     * @param y Imaginary Y part.
     * @param z Imaginary Z part.
     */
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    /**
     * @brief Returns the identity quaternion.
     * @return The identity quaternion.
     */
    static Quaternion identity();

    /**
     * @brief Builds a quaternion from an axis and angle.
     * @param axis Rotation axis (does not need to be normalized).
     * @param angle_rad Rotation angle in radians.
     * @return The rotation quaternion.
     */
    static Quaternion from_axis_angle(const Vec3& axis, double angle_rad);

    /**
     * @brief Builds a quaternion from Euler angles.
     * @param roll Roll angle in radians.
     * @param pitch Pitch angle in radians.
     * @param yaw Yaw angle in radians.
     * @return The rotation quaternion.
     */
    static Quaternion from_euler(double roll, double pitch, double yaw);

    /**
     * @brief Hamilton product with another quaternion.
     * @param q The other quaternion.
     * @return The composed rotation.
     */
    Quaternion operator*(const Quaternion& q) const;

    /**
     * @brief Rotates a vector by this quaternion.
     * @param v The vector to rotate.
     * @return The rotated vector.
     */
    Vec3 rotate(const Vec3& v) const;

    /**
     * @brief Returns the conjugate quaternion.
     * @return The conjugate (inverse rotation for unit quaternions).
     */
    Quaternion conjugate() const;

    /**
     * @brief Returns a unit-length version of this quaternion.
     * @return The normalized quaternion.
     */
    Quaternion normalized() const;

    /**
     * @brief Converts to roll, pitch, yaw angles.
     * @return Euler angles (roll, pitch, yaw) in radians.
     */
    Vec3 to_euler() const;  // returns roll, pitch, yaw in radians
};

}  // namespace robcraft::engine::math
