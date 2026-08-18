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

#include "robcraft/engine/world/terrain_mesh.hpp"

#include <algorithm>
#include <cmath>

#include "robcraft/engine/world/terrain.hpp"

namespace robcraft::engine::world {

using namespace robcraft::engine::world;

static float height_at_vert(const Terrain& t, int cx, int cz) {
    if (cx < 0 || cx >= t.width() || cz < 0 || cz >= t.height()) return 0.0f;
    return t.height_at(cx, cz);
}

static Vec3 vertex_pos(const Terrain& t, int cx, int cz) {
    double half = t.width() * t.cell_size() * 0.5;
    double y = height_at_vert(t, cx, cz);
    return Vec3(cx * t.cell_size() - half, y, cz * t.cell_size() - half);
}

static Vec3 vertex_normal(const Terrain& t, int cx, int cz) {
    float hl = height_at_vert(t, cx - 1, cz);
    float hr = height_at_vert(t, cx + 1, cz);
    float hd = height_at_vert(t, cx, cz - 1);
    float hu = height_at_vert(t, cx, cz + 1);

    Vec3 right(2.0 * t.cell_size(), static_cast<double>(hr - hl), 0.0);
    Vec3 down(0.0, static_cast<double>(hu - hd), 2.0 * t.cell_size());
    return down.cross(right).normalized();
}

static void terrain_type_color(TerrainType t, float& r, float& g, float& b) {
    switch (t) {
        case TerrainType::Grass:
            r = 0.20f;
            g = 0.60f;
            b = 0.12f;
            break;
        case TerrainType::Dirt:
            r = 0.60f;
            g = 0.40f;
            b = 0.18f;
            break;
        case TerrainType::Rock:
            r = 0.45f;
            g = 0.42f;
            b = 0.38f;
            break;
        case TerrainType::Sand:
            r = 0.75f;
            g = 0.70f;
            b = 0.40f;
            break;
        case TerrainType::Snow:
            r = 0.88f;
            g = 0.88f;
            b = 0.88f;
            break;
        default:
            r = 0.3f;
            g = 0.3f;
            b = 0.3f;
            break;
    }
}

static int type_to_layer(TerrainType t) {
    switch (t) {
        case TerrainType::Grass:
            return 0;
        case TerrainType::Dirt:
            return 1;
        case TerrainType::Rock:
            return 2;
        case TerrainType::Sand:
            return 3;
        case TerrainType::Snow:
            return 4;
        default:
            return 0;
    }
}

// 4 neighbor cells around vertex (cx, cz): (cx,cz), (cx-1,cz), (cx,cz-1), (cx-1,cz-1)
static void splat_weights_at(const Terrain& t, int cx, int cz, float out[4]) {
    const int dx[4] = {0, -1, 0, -1};
    const int dz[4] = {0, 0, -1, -1};
    int counts[5] = {0, 0, 0, 0, 0};
    for (int k = 0; k < 4; ++k) {
        int nx = cx + dx[k], nz = cz + dz[k];
        if (nx < 0 || nx >= t.width() || nz < 0 || nz >= t.height()) continue;
        counts[type_to_layer(t.terrain_type(nx, nz))]++;
    }
    int total = 0;
    for (int i = 0; i < 5; ++i) total += counts[i];
    out[0] = total > 0 ? static_cast<float>(counts[0]) / total : 1.0f;
    out[1] = total > 0 ? static_cast<float>(counts[1]) / total : 0.0f;
    out[2] = total > 0 ? static_cast<float>(counts[2]) / total : 0.0f;
    out[3] = total > 0 ? static_cast<float>(counts[3]) / total : 0.0f;
}

static void add_vertex(TerrainMeshData& d, const Vec3& p, const Vec3& n, float r, float g, float b,
                       double uv_x = 0.0, double uv_z = 0.0) {
    d.vertices.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                          static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z),
                          r, g, b, static_cast<float>(uv_x), static_cast<float>(uv_z), 1.0f, 0.0f,
                          0.0f});
}

TerrainMeshData build_terrain_mesh(const Terrain& terrain) {
    TerrainMeshData data;
    int w = terrain.width();
    int h = terrain.height();

    for (int z = 0; z < h; ++z) {
        for (int x = 0; x < w; ++x) {
            Vec3 p = vertex_pos(terrain, x, z);
            Vec3 n = vertex_normal(terrain, x, z);
            float r, g, b;
            terrain_type_color(terrain.terrain_type(x, z), r, g, b);
            float splat[4];
            splat_weights_at(terrain, x, z, splat);
            data.weights.insert(data.weights.end(), splat, splat + 4);
            double hx = w * terrain.cell_size() * 0.5;
            double hz = h * terrain.cell_size() * 0.5;
            add_vertex(data, p, n, r, g, b,
                       (x * terrain.cell_size() - hx) / terrain.texture_repeat(),
                       (z * terrain.cell_size() - hz) / terrain.texture_repeat());
        }
    }

    for (int z = 0; z < h - 1; ++z) {
        for (int x = 0; x < w - 1; ++x) {
            unsigned int a = static_cast<unsigned int>(z * w + x);
            unsigned int b = a + 1;
            unsigned int c = a + w;
            unsigned int d = c + 1;
            data.indices.push_back(a);
            data.indices.push_back(c);
            data.indices.push_back(b);
            data.indices.push_back(b);
            data.indices.push_back(c);
            data.indices.push_back(d);
        }
    }

    return data;
}

TerrainMeshData build_terrain_grid(const Terrain& terrain) {
    TerrainMeshData data;
    int w = terrain.width(), h = terrain.height();
    if (w == 0 || h == 0) return data;
    const float gr = 0.35f, gg = 0.35f, gb = 0.35f;
    double half = w * terrain.cell_size() * 0.5;
    auto vertex = [&](int cx, int cz) {
        int ccx = std::min(std::max(cx, 0), w - 1);
        int ccz = std::min(std::max(cz, 0), h - 1);
        float y = terrain.heights()[terrain.index(ccx, ccz)];
        return Vec3(cx * terrain.cell_size() - half, y, cz * terrain.cell_size() - half);
    };
    for (int k = 0; k <= w; ++k) {
        for (int z = 0; z < h; ++z) {
            Vec3 p0 = vertex(k, z), p1 = vertex(k, z + 1);
            add_vertex(data, p0, Vec3(0, 1, 0), gr, gg, gb);
            add_vertex(data, p1, Vec3(0, 1, 0), gr, gg, gb);
        }
    }
    for (int k = 0; k <= h; ++k) {
        for (int x = 0; x < w; ++x) {
            Vec3 p0 = vertex(x, k), p1 = vertex(x + 1, k);
            add_vertex(data, p0, Vec3(0, 1, 0), gr, gg, gb);
            add_vertex(data, p1, Vec3(0, 1, 0), gr, gg, gb);
        }
    }
    for (size_t i = 0; i < data.vertices.size(); ++i) {
        data.indices.push_back(static_cast<unsigned int>(i));
    }
    return data;
}

TerrainMeshData build_terrain_water_mesh(const Terrain& terrain) {
    TerrainMeshData data;
    int w = terrain.width(), h = terrain.height();
    if (w == 0 || h == 0) return data;
    double half = w * terrain.cell_size() * 0.5;
    const int max_search = 4;
    for (int z = 0; z < h; ++z) {
        for (int x = 0; x < w; ++x) {
            if (!terrain.has_water(x, z)) continue;

            // Shore distance: meters to the nearest dry cell, capped at max_search cells.
            float shore = static_cast<float>(max_search) * static_cast<float>(terrain.cell_size());
            for (int r = 1; r <= max_search; ++r) {
                bool found_dry = false;
                for (int dz = -r; dz <= r && !found_dry; ++dz) {
                    for (int dx = -r; dx <= r && !found_dry; ++dx) {
                        if (std::abs(dx) != r && std::abs(dz) != r) continue;  // ring only
                        int nx = x + dx, nz = z + dz;
                        if (nx < 0 || nx >= w || nz < 0 || nz >= h) continue;
                        if (terrain.water_[terrain.index(nx, nz)] <= Terrain::WATER_OFF) {
                            found_dry = true;
                        }
                    }
                }
                if (found_dry) {
                    shore = static_cast<float>(r) * static_cast<float>(terrain.cell_size());
                    break;
                }
            }

            // Clamp height sampling to in-bounds vertices so the last row/column of
            // water cells reads the true edge height instead of an OOB 0.0. Geometry
            // coordinates below stay unclamped (the quad is still one cell wide).
            int cx1 = std::min(x + 1, w - 1);
            int cz1 = std::min(z + 1, h - 1);
            double x0 = x * terrain.cell_size() - half;
            double x1 = (x + 1) * terrain.cell_size() - half;
            double z0 = z * terrain.cell_size() - half;
            double z1 = (z + 1) * terrain.cell_size() - half;
            unsigned int base = static_cast<unsigned int>(data.vertices.size());
            // Per-corner terrain heights make the surface follow the terrain slope;
            // each corner also carries its terrain height in the v UV channel.
            float h00 = height_at_vert(terrain, x, z);
            float h10 = height_at_vert(terrain, cx1, z);
            float h11 = height_at_vert(terrain, cx1, cz1);
            float h01 = height_at_vert(terrain, x, cz1);
            add_vertex(data, Vec3(x0, h00 + Terrain::WATER_SURFACE_OFFSET, z0), Vec3(0, 1, 0),
                       0.30f, 0.55f, 0.95f, shore, static_cast<double>(h00));
            add_vertex(data, Vec3(x1, h10 + Terrain::WATER_SURFACE_OFFSET, z0), Vec3(0, 1, 0),
                       0.30f, 0.55f, 0.95f, shore, static_cast<double>(h10));
            add_vertex(data, Vec3(x1, h11 + Terrain::WATER_SURFACE_OFFSET, z1), Vec3(0, 1, 0),
                       0.30f, 0.55f, 0.95f, shore, static_cast<double>(h11));
            add_vertex(data, Vec3(x0, h01 + Terrain::WATER_SURFACE_OFFSET, z1), Vec3(0, 1, 0),
                       0.30f, 0.55f, 0.95f, shore, static_cast<double>(h01));
            // Match the terrain mesh's diagonal (b-c): triangles (a,c,b),(b,c,d),
            // so the two surfaces split identically and terrain never pokes through.
            data.indices.push_back(base);
            data.indices.push_back(base + 3);
            data.indices.push_back(base + 1);
            data.indices.push_back(base + 1);
            data.indices.push_back(base + 3);
            data.indices.push_back(base + 2);
        }
    }
    return data;
}

}  // namespace robcraft::engine::world
