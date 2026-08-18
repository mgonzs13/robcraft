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

#include "robcraft/engine/math/frame_conversion.hpp"

#include <cmath>

#include "robcraft/engine/math/constants.hpp"

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

Vec3 sim_to_rep103_position(const Vec3& sim_pos) {
    return Vec3(sim_pos.z, sim_pos.x, sim_pos.y);
}

Quaternion sim_to_rep103_orientation(const Quaternion& sim_rot) {
    // Basis change from sim (Y up, Z forward, X left) to REP-103 (X forward,
    // Y left, Z up). The axis permutation X->Y, Y->Z, Z->X is a 120 deg
    // rotation about the normalized (1,1,1) axis.
    static const Quaternion kFrameChange = Quaternion::from_axis_angle(
        Vec3(1.0, 1.0, 1.0).normalized(), robcraft::engine::math::kTwoPi / 3.0);
    return (kFrameChange * sim_rot * kFrameChange.conjugate()).normalized();
}

}  // namespace robcraft::engine::math
