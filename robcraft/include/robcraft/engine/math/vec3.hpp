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

#include <cmath>

namespace robcraft::engine::math {
using namespace robcraft::engine::math;

/** @brief 3D double-precision vector with component-wise arithmetic. */
struct Vec3 {
    /** @brief X component. */
    double x = 0.0;
    /** @brief Y component. */
    double y = 0.0;
    /** @brief Z component. */
    double z = 0.0;

    Vec3() = default;

    /**
     * @brief Constructs a vector from explicit components.
     * @param x X component.
     * @param y Y component.
     * @param z Z component.
     */
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    /**
     * @brief Component-wise addition.
     * @param o The other vector.
     * @return The sum vector.
     */
    Vec3 operator+(const Vec3& o) const { return {this->x + o.x, this->y + o.y, this->z + o.z}; }

    /**
     * @brief Component-wise subtraction.
     * @param o The other vector.
     * @return The difference vector.
     */
    Vec3 operator-(const Vec3& o) const { return {this->x - o.x, this->y - o.y, this->z - o.z}; }

    /**
     * @brief Uniform scalar multiplication.
     * @param s The scale factor.
     * @return The scaled vector.
     */
    Vec3 operator*(double s) const { return {this->x * s, this->y * s, this->z * s}; }

    /**
     * @brief Uniform scalar division.
     * @param s The divisor.
     * @return The scaled vector.
     */
    Vec3 operator/(double s) const { return {this->x / s, this->y / s, this->z / s}; }

    /**
     * @brief Unary negation.
     * @return The negated vector.
     */
    Vec3 operator-() const { return {-this->x, -this->y, -this->z}; }

    /**
     * @brief Component-wise add-assign.
     * @param o The other vector.
     * @return Reference to this vector.
     */
    Vec3& operator+=(const Vec3& o) {
        this->x += o.x;
        this->y += o.y;
        this->z += o.z;
        return *this;
    }

    /**
     * @brief Component-wise subtract-assign.
     * @param o The other vector.
     * @return Reference to this vector.
     */
    Vec3& operator-=(const Vec3& o) {
        this->x -= o.x;
        this->y -= o.y;
        this->z -= o.z;
        return *this;
    }

    /**
     * @brief Uniform scalar multiply-assign.
     * @param s The scale factor.
     * @return Reference to this vector.
     */
    Vec3& operator*=(double s) {
        this->x *= s;
        this->y *= s;
        this->z *= s;
        return *this;
    }

    /**
     * @brief Checks component-wise equality.
     * @param o The other vector.
     * @return True if all components match.
     */
    bool operator==(const Vec3& o) const {
        return this->x == o.x && this->y == o.y && this->z == o.z;
    }

    /**
     * @brief Checks component-wise inequality.
     * @param o The other vector.
     * @return True if any component differs.
     */
    bool operator!=(const Vec3& o) const { return !(*this == o); }

    /**
     * @brief Squared length of the vector.
     * @return The squared magnitude.
     */
    double length_sq() const { return this->x * this->x + this->y * this->y + this->z * this->z; }

    /**
     * @brief Length of the vector.
     * @return The magnitude.
     */
    double length() const { return std::sqrt(this->length_sq()); }

    /**
     * @brief Returns a unit vector in the same direction.
     * @return The normalized vector, or zero if the length is zero.
     */
    Vec3 normalized() const {
        double len = this->length();
        return len > 0.0 ? *this / len : Vec3{};
    }

    /**
     * @brief Computes the dot product with another vector.
     * @param o The other vector.
     * @return The dot product.
     */
    double dot(const Vec3& o) const { return this->x * o.x + this->y * o.y + this->z * o.z; }

    /**
     * @brief Computes the cross product with another vector.
     * @param o The other vector.
     * @return The cross product.
     */
    Vec3 cross(const Vec3& o) const {
        return {this->y * o.z - this->z * o.y, this->z * o.x - this->x * o.z,
                this->x * o.y - this->y * o.x};
    }
};

/**
 * @brief Scalar-times-vector multiplication.
 * @param s The scale factor.
 * @param v The vector.
 * @return The scaled vector.
 */
inline Vec3 operator*(double s, const Vec3& v) {
    return v * s;
}

}  // namespace robcraft::engine::math
