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

#include "robcraft/renderer/reflection.hpp"

#include "robcraft/engine/math/mat4.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::math;

Mat4 reflected_view(const Mat4& view, float plane_y) {
    // Post-multiply by the mirror matrix about the plane y = plane_y:
    // [1 0 0  0]
    // [0 -1 0 2y]
    // [0 0 1  0]
    // [0 0 0  1]  (column-major: data[col*4 + row]).
    // So reflected * p == view * (mirror * p): the reflected camera sees the
    // virtual (mirrored) world exactly as the original camera would, which is
    // what a mirror displays. Column 1 is negated and column 3 gains
    // 2*plane_y * column 1.
    Mat4 r = view;
    for (int i = 0; i < 4; ++i) {
        r.data[4 + i] = -view.data[4 + i];
        r.data[12 + i] = view.data[12 + i] + 2.0f * plane_y * view.data[4 + i];
    }
    return r;
}

std::array<float, 4> reflection_clip_plane(float plane_y, float eps) {
    return {0.0f, 1.0f, 0.0f, -(plane_y - eps)};
}

}  // namespace robcraft::renderer
