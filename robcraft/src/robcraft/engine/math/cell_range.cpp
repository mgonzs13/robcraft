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

#include "robcraft/engine/math/cell_range.hpp"

#include <algorithm>
#include <cstdlib>

namespace robcraft::engine::math {

using namespace robcraft::engine::math;

CellRange wall_run_cells(int ax, int az, int bx, int bz) {
    int dx = std::abs(bx - ax);
    int dz = std::abs(bz - az);
    if (dz > dx) {
        return {ax, std::min(az, bz), ax, std::max(az, bz)};
    }
    return {std::min(ax, bx), az, std::max(ax, bx), az};
}

CellRange floor_rect_cells(int ax, int az, int bx, int bz) {
    return {std::min(ax, bx), std::min(az, bz), std::max(ax, bx), std::max(az, bz)};
}

bool cell_range_is_horizontal(const CellRange& r) {
    return r.z0 == r.z1;
}

bool cell_ranges_merge(const CellRange& a, const CellRange& b) {
    if (a.z0 == a.z1 && b.z0 == b.z1 && a.z0 == b.z0) {
        return a.x0 <= b.x1 + 1 && b.x0 <= a.x1 + 1;
    }
    if (a.x0 == a.x1 && b.x0 == b.x1 && a.x0 == b.x0) {
        return a.z0 <= b.z1 + 1 && b.z0 <= a.z1 + 1;
    }
    return false;
}

}  // namespace robcraft::engine::math
