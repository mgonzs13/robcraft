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
#include <cstdint>

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

/** @brief 2D double-precision vector with component-wise arithmetic. */
struct Vec2 {
    /** @brief X component. */
    double x = 0.0;
    /** @brief Y component. */
    double y = 0.0;

    Vec2() = default;

    /**
     * @brief Constructs a vector from explicit components.
     * @param x X component.
     * @param y Y component.
     */
    Vec2(double x, double y) : x(x), y(y) {}

    /**
     * @brief Component-wise addition.
     * @param o The other vector.
     * @return The sum vector.
     */
    Vec2 operator+(const Vec2& o) const { return {this->x + o.x, this->y + o.y}; }

    /**
     * @brief Component-wise subtraction.
     * @param o The other vector.
     * @return The difference vector.
     */
    Vec2 operator-(const Vec2& o) const { return {this->x - o.x, this->y - o.y}; }

    /**
     * @brief Uniform scalar multiplication.
     * @param s The scale factor.
     * @return The scaled vector.
     */
    Vec2 operator*(double s) const { return {this->x * s, this->y * s}; }

    /**
     * @brief Uniform scalar division.
     * @param s The divisor.
     * @return The scaled vector.
     */
    Vec2 operator/(double s) const { return {this->x / s, this->y / s}; }

    /**
     * @brief Component-wise add-assign.
     * @param o The other vector.
     * @return Reference to this vector.
     */
    Vec2& operator+=(const Vec2& o) {
        this->x += o.x;
        this->y += o.y;
        return *this;
    }

    /**
     * @brief Component-wise subtract-assign.
     * @param o The other vector.
     * @return Reference to this vector.
     */
    Vec2& operator-=(const Vec2& o) {
        this->x -= o.x;
        this->y -= o.y;
        return *this;
    }

    /**
     * @brief Uniform scalar multiply-assign.
     * @param s The scale factor.
     * @return Reference to this vector.
     */
    Vec2& operator*=(double s) {
        this->x *= s;
        this->y *= s;
        return *this;
    }

    /**
     * @brief Checks component-wise equality.
     * @param o The other vector.
     * @return True if both components match.
     */
    bool operator==(const Vec2& o) const { return this->x == o.x && this->y == o.y; }

    /**
     * @brief Checks component-wise inequality.
     * @param o The other vector.
     * @return True if any component differs.
     */
    bool operator!=(const Vec2& o) const { return !(*this == o); }

    /**
     * @brief Squared length of the vector.
     * @return The squared magnitude.
     */
    double length_sq() const { return this->x * this->x + this->y * this->y; }

    /**
     * @brief Length of the vector.
     * @return The magnitude.
     */
    double length() const { return std::sqrt(this->length_sq()); }

    /**
     * @brief Returns a unit vector in the same direction.
     * @return The normalized vector, or zero if the length is zero.
     */
    Vec2 normalized() const {
        double len = this->length();
        return len > 0.0 ? *this / len : Vec2{};
    }

    /**
     * @brief Computes the dot product with another vector.
     * @param o The other vector.
     * @return The dot product.
     */
    double dot(const Vec2& o) const { return this->x * o.x + this->y * o.y; }
};

/**
 * @brief Scalar-times-vector multiplication.
 * @param s The scale factor.
 * @param v The vector.
 * @return The scaled vector.
 */
inline Vec2 operator*(double s, const Vec2& v) {
    return v * s;
}

}  // namespace robcraft::engine::math
