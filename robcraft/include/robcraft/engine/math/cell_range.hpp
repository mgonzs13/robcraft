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

/** @brief Inclusive, normalized cell rectangle (x0<=x1, z0<=z1). */
struct CellRange {
    /** @brief Minimum column index. */
    int x0 = 0;
    /** @brief Minimum row index. */
    int z0 = 0;
    /** @brief Maximum column index. */
    int x1 = 0;
    /** @brief Maximum row index. */
    int z1 = 0;
};

/**
 * @brief Computes a one-cell-thick wall run between two cells along the dominant axis.
 * @param ax Anchor column.
 * @param az Anchor row.
 * @param bx Current column.
 * @param bz Current row.
 * @return Normalized range spanning the run; ties on distance favor the horizontal axis.
 */
CellRange wall_run_cells(int ax, int az, int bx, int bz);

/**
 * @brief Computes the full rectangle between two cells (for floors).
 * @param ax Anchor column.
 * @param az Anchor row.
 * @param bx Current column.
 * @param bz Current row.
 * @return Normalized range covering both corners.
 */
CellRange floor_rect_cells(int ax, int az, int bx, int bz);

/**
 * @brief Checks whether a range spans exactly one row (axis-aligned along x).
 * @param r The range.
 * @return True if the range is a single row.
 */
bool cell_range_is_horizontal(const CellRange& r);

/**
 * @brief Checks whether two runs on the same axis overlap or abut.
 * @param a First run.
 * @param b Second run.
 * @return True if both lie on the same row or column and their ranges overlap or touch.
 */
bool cell_ranges_merge(const CellRange& a, const CellRange& b);

}  // namespace robcraft::engine::math
