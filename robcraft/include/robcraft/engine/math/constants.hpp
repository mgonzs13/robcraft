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

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

/** @brief Ratio of a circle's circumference to its diameter. */
constexpr double kPi = 3.14159265358979323846;

/** @brief Two times pi. */
constexpr double kTwoPi = 2.0 * kPi;

/** @brief Converts an angle from degrees to radians.
 *  @param deg Angle in degrees.
 *  @return Angle in radians. */
constexpr double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
}

/** @brief Converts an angle from radians to degrees.
 *  @param rad Angle in radians.
 *  @return Angle in degrees. */
constexpr double rad_to_deg(double rad) {
    return rad * 180.0 / kPi;
}

}  // namespace robcraft::engine::math
