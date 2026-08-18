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
#include <optional>
#include <string>
#include <vector>

#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/engine/world/terrain_mesh.hpp"

namespace robcraft::engine::world {

using namespace robcraft::engine::world;
using namespace robcraft::engine::collision;
using namespace robcraft::engine::math;

/** @brief Terrain surface material type per cell. */
enum class TerrainType : uint8_t {
    /** @brief Grass surface. */
    Grass = 0,
    /** @brief Dirt surface. */
    Dirt = 1,
    /** @brief Rocky surface. */
    Rock = 2,
    /** @brief Sandy surface. */
    Sand = 3,
    /** @brief Snowy surface. */
    Snow = 4,
    /** @brief Sentinel value equal to the number of terrain types. */
    Count = 5
};

/** @brief Heightmap terrain grid with editing, raycasting, and serialization. */
class Terrain {
public:
    /** @brief Constructs an empty terrain. */
    Terrain();
    /**
     * @brief Constructs a flat terrain of the given dimensions.
     * @param width Number of cells along x.
     * @param height Number of cells along z.
     * @param cell_size World-space edge length of each cell.
     */
    Terrain(int width, int height, double cell_size = 1.0);

    /** @brief Number of cells along x. */
    int width() const { return this->width_; }
    /** @brief Number of cells along z. */
    int height() const { return this->height_; }
    /** @brief World-space edge length of each cell. */
    double cell_size() const { return this->cell_size_; }
    /** @brief World-space meters per texture repeat along X/Z.
     *  @return Repeat distance in meters. */
    double texture_repeat() const { return this->texture_repeat_; }
    /** @brief Sets the world-space texture repeat distance.
     *  @param meters New repeat distance in meters. */
    void set_texture_repeat(double meters) { this->texture_repeat_ = meters; }

    /**
     * @brief Returns the height of a cell.
     * @param cx Column index.
     * @param cz Row index.
     * @return Cell height, or 0 if out of bounds.
     */
    float height_at(int cx, int cz) const;
    /**
     * @brief Returns the bilinearly interpolated height at a world position.
     * @param wx World x coordinate.
     * @param wz World z coordinate.
     * @return Interpolated height, or 0 if out of bounds.
     */
    float height_at_world(double wx, double wz) const;
    /**
     * @brief Sets the height of a cell and marks the terrain dirty.
     * @param cx Column index.
     * @param cz Row index.
     * @param h New height value.
     */
    void set_height(int cx, int cz, float h);

    /**
     * @brief Checks whether a cell is walkable.
     * @param cx Column index.
     * @param cz Row index.
     * @return True if walkable and in bounds.
     */
    bool is_walkable(int cx, int cz) const;
    /**
     * @brief Sets the walkable flag of a cell.
     * @param cx Column index.
     * @param cz Row index.
     * @param w New walkable flag.
     */
    void set_walkable(int cx, int cz, bool w);

    /**
     * @brief Raycasts against the terrain surface.
     * @param ray The ray to test.
     * @return Distance to first terrain hit, or nullopt if none.
     */
    std::optional<double> raycast(const Ray& ray) const;

    /**
     * @brief Returns the world-space center of a cell.
     * @param cx Column index.
     * @param cz Row index.
     * @return Center position including terrain height.
     */
    Vec3 cell_center_world(int cx, int cz) const;
    /**
     * @brief Converts a world position to terrain cell coordinates.
     * @param wx World x coordinate.
     * @param wz World z coordinate.
     * @param cx Out-param receiving the column index.
     * @param cz Out-param receiving the row index.
     */
    void world_to_cell(double wx, double wz, int& cx, int& cz) const;

    /**
     * @brief Raises heights in a brush region around a cell.
     * @param cx Center column index.
     * @param cz Center row index.
     * @param amount Maximum height increase at the brush center.
     * @param brush_radius Brush radius in cells.
     */
    void raise(int cx, int cz, float amount, int brush_radius);
    /**
     * @brief Lowers heights in a brush region around a cell.
     * @param cx Center column index.
     * @param cz Center row index.
     * @param amount Maximum height decrease at the brush center.
     * @param brush_radius Brush radius in cells.
     */
    void lower(int cx, int cz, float amount, int brush_radius);
    /**
     * @brief Smooths heights toward a target value in a brush region.
     * @param cx Center column index.
     * @param cz Center row index.
     * @param target Target height to blend toward.
     * @param brush_radius Brush radius in cells.
     */
    void flatten(int cx, int cz, float target, int brush_radius);

    /**
     * @brief Returns the cliff level of a cell.
     * @param cx Column index.
     * @param cz Row index.
     * @return Cliff level, or 0 if out of bounds.
     */
    uint8_t cliff_level(int cx, int cz) const;
    /**
     * @brief Sets the cliff level of a cell and marks the terrain dirty.
     * @param cx Column index.
     * @param cz Row index.
     * @param level New cliff level.
     */
    void set_cliff_level(int cx, int cz, uint8_t level);
    /**
     * @brief Snaps every cell in a brush region to a target cliff level.
     * @param cx Center column index.
     * @param cz Center row index.
     * @param level Target cliff level, clamped to [0, 16].
     * @param brush_radius Brush radius in cells.
     */
    void cliff_to_level(int cx, int cz, uint8_t level, int brush_radius);

    /**
     * @brief Returns the terrain type of a cell.
     * @param cx Column index.
     * @param cz Row index.
     * @return Terrain type, or Grass if out of bounds.
     */
    TerrainType terrain_type(int cx, int cz) const;
    /**
     * @brief Sets the terrain type of a cell and marks the terrain dirty.
     * @param cx Column index.
     * @param cz Row index.
     * @param type New terrain type.
     */
    void set_terrain_type(int cx, int cz, TerrainType type);
    /**
     * @brief Paints a terrain type over a brush region.
     * @param cx Center column index.
     * @param cz Center row index.
     * @param type Terrain type to paint.
     * @param brush_radius Brush radius in cells.
     */
    void paint_type(int cx, int cz, TerrainType type, int brush_radius);

    /** @brief Sentinel height meaning "no water". */
    static constexpr float WATER_OFF = -999.0f;
    /** @brief Water surface sits this many meters above the terrain height. */
    static constexpr float WATER_SURFACE_OFFSET = 0.5f;
    /** @brief Height added to a cell per cliff level. */
    static constexpr float CLIFF_HEIGHT = 2.0f;
    /** @brief Maximum cliff level. */
    static constexpr uint8_t MAX_CLIFF_LEVEL = 16;

    /**
     * @brief Whether a cell has water.
     * @param cx Column index.
     * @param cz Row index.
     * @return True if the cell is wet.
     */
    bool has_water(int cx, int cz) const;
    /**
     * @brief Returns the water surface height of a cell, derived from terrain.
     * @param cx Column index.
     * @param cz Row index.
     * @return Terrain height + WATER_SURFACE_OFFSET, or WATER_OFF if dry.
     */
    float water_surface_height(int cx, int cz) const;
    /**
     * @brief Mean water surface height across all wet cells.
     * @return Average of water_surface_height() over wet cells, or WATER_OFF if none.
     */
    float water_plane_height() const;
    /**
     * @brief Sets whether a cell has water.
     * @param cx Column index.
     * @param cz Row index.
     * @param present True to place water, false to clear.
     */
    void set_water(int cx, int cz, bool present);
    /**
     * @brief Paints water presence over a brush region.
     * @param cx Center column index.
     * @param cz Center row index.
     * @param present True to place water, false to clear.
     * @param brush_radius Brush radius in cells.
     */
    void paint_water(int cx, int cz, bool present, int brush_radius);

    /** @brief Whether the terrain has unsaved modifications. */
    bool dirty() const { return this->dirty_; }
    /** @brief Clears the dirty flag. */
    void clear_dirty() { this->dirty_ = false; }

    /** @brief Height of every cell in row-major order. */
    const std::vector<float>& heights() const { return this->heights_; }

    /**
     * @brief Checks whether cell coordinates are inside the grid.
     * @param cx Column index.
     * @param cz Row index.
     * @return True if in bounds.
     */
    bool in_bounds(int cx, int cz) const;

private:
    /** @brief Grants mesh-generation access to the private storage helpers. */
    friend TerrainMeshData build_terrain_mesh(const Terrain& terrain);
    /** @brief Grants grid-generation access to the private storage helpers. */
    friend TerrainMeshData build_terrain_grid(const Terrain& terrain);
    /** @brief Grants water-mesh-generation access to the private storage helpers. */
    friend TerrainMeshData build_terrain_water_mesh(const Terrain& terrain);
    /** @brief Grants terrain-loading access to the private storage vectors. */
    friend bool load_terrain(Terrain& terrain, const std::string& path);
    /** @brief Grants terrain-saving access to the private storage vectors. */
    friend bool save_terrain(const Terrain& terrain, const std::string& path);

    /**
     * @brief Converts cell coordinates to a flat row-major index.
     * @param cx Column index.
     * @param cz Row index.
     * @return Flat index into the storage vectors.
     */
    int index(int cx, int cz) const { return cz * this->width_ + cx; }

    /**
     * @brief Visits every cell in a radial brush region.
     * @param cx Center column index.
     * @param cz Center row index.
     * @param brush_radius Brush radius in cells.
     * @param fn Callback invoked as fn(c, r, dist) with the cell's center
     *        distance from (cx, cz); out-of-bounds cells are skipped.
     */
    template <typename F>
    void for_each_brush_cell(int cx, int cz, int brush_radius, F&& fn) {
        for (int r = cz - brush_radius; r <= cz + brush_radius; ++r) {
            for (int c = cx - brush_radius; c <= cx + brush_radius; ++c) {
                if (!this->in_bounds(c, r)) continue;
                int dx = c - cx, dz = r - cz;
                float dist = std::sqrt(static_cast<float>(dx * dx + dz * dz));
                if (dist <= brush_radius) fn(c, r, dist);
            }
        }
    }

    /**
     * @brief Returns the height of a cell without bounds checking.
     * @param cx Column index.
     * @param cz Row index.
     * @return Cell height, or 0 if out of bounds.
     */
    float height_at_vert(int cx, int cz) const;
    /** @brief Marks the terrain as having unsaved modifications. */
    void mark_dirty() { this->dirty_ = true; }

    /** @brief Number of cells along x. */
    int width_ = 0;
    /** @brief Number of cells along z. */
    int height_ = 0;
    /** @brief World-space edge length of each cell. */
    double cell_size_ = 1.0;
    /** @brief Per-cell heights in row-major order. */
    std::vector<float> heights_;
    /** @brief Per-cell walkability in row-major order. */
    std::vector<uint8_t> walkable_;
    /** @brief Per-cell cliff levels in row-major order. */
    std::vector<uint8_t> cliff_level_;
    /** @brief Per-cell terrain types in row-major order. */
    std::vector<uint8_t> terrain_type_;
    /** @brief Per-cell water presence in row-major order; WATER_OFF = dry, present = wet. */
    std::vector<float> water_;
    /** @brief World-space texture repeat distance in meters. */
    double texture_repeat_ = 4.0;
    /** @brief Whether the terrain has unsaved modifications. */
    bool dirty_ = true;
};

}  // namespace robcraft::engine::world
