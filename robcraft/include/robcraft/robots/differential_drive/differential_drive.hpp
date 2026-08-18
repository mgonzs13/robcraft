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

namespace robcraft::robots::differential_drive {

/** @brief Differential-drive kinematic model computing linear and angular velocity. */
struct DifferentialDrive {
    /** @brief Distance between the two drive wheels in meters. */
    double wheel_base = 0.42;
    /** @brief Current left wheel velocity in m/s. */
    double left_velocity = 0.0;
    /** @brief Current right wheel velocity in m/s. */
    double right_velocity = 0.0;
    /** @brief Maximum achievable linear speed in m/s. */
    double max_linear_speed = 1.0;
    /** @brief Odometry publish rate in Hz. */
    double odom_rate = 30.0;
    /** @brief Standard deviation of odometry position noise in meters. */
    double odom_noise_stddev = 0.0;

    /**
     * @brief Computes the linear velocity from the two wheel speeds.
     * @return Linear velocity in m/s.
     */
    double linear_velocity() const { return (this->right_velocity + this->left_velocity) / 2.0; }

    /**
     * @brief Computes the angular velocity from the two wheel speeds.
     * @return Angular velocity in rad/s.
     */
    double angular_velocity() const {
        return (this->right_velocity - this->left_velocity) / this->wheel_base;
    }
};

}  // namespace robcraft::robots::differential_drive
