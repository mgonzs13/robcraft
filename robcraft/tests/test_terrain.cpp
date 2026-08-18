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

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/engine/world/terrain_io.hpp"
#include "robcraft/engine/world/terrain_mesh.hpp"

using namespace robcraft::engine::collision;
using namespace robcraft::engine::world;
using Catch::Approx;

TEST_CASE("Terrain default construction", "[terrain]") {
    robcraft::engine::world::Terrain t(10, 10, 1.0);

    REQUIRE(t.width() == 10);
    REQUIRE(t.height() == 10);
    REQUIRE(t.cell_size() == Approx(1.0));
}

TEST_CASE("Terrain heights and walkability", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 1.0);

    SECTION("default is walkable and flat") {
        REQUIRE(t.is_walkable(3, 3));
        REQUIRE(t.height_at(3, 3) == Approx(0.0f));
    }

    SECTION("set and retrieve height") {
        t.set_height(2, 4, 5.0f);
        REQUIRE(t.height_at(2, 4) == Approx(5.0f));
    }

    SECTION("set walkability") {
        REQUIRE(t.is_walkable(1, 1));
        t.set_walkable(1, 1, false);
        REQUIRE(!t.is_walkable(1, 1));
    }

    SECTION("out of bounds returns default") {
        REQUIRE(!t.is_walkable(-1, 0));
        REQUIRE(t.height_at(-1, 0) == Approx(0.0f));
    }
}

TEST_CASE("Terrain world_to_cell", "[terrain]") {
    robcraft::engine::world::Terrain t(64, 64, 1.0);
    int cx, cz;

    t.world_to_cell(0.0, 0.0, cx, cz);
    REQUIRE(cx == 32);
    REQUIRE(cz == 32);

    t.world_to_cell(-32.0, -32.0, cx, cz);
    REQUIRE(cx == 0);
    REQUIRE(cz == 0);

    t.world_to_cell(31.5, 31.5, cx, cz);
    REQUIRE(cx == 63);
    REQUIRE(cz == 63);
}

TEST_CASE("Terrain cell_center_world", "[terrain]") {
    robcraft::engine::world::Terrain t(64, 64, 1.0);

    auto center = t.cell_center_world(32, 32);
    REQUIRE(center.x == Approx(0.5));
    REQUIRE(center.z == Approx(0.5));
}

TEST_CASE("Terrain brush operations", "[terrain]") {
    robcraft::engine::world::Terrain t(20, 20, 1.0);

    SECTION("raise modifies cells within radius") {
        t.raise(10, 10, 2.0f, 3);
        REQUIRE(t.height_at(10, 10) > 1.0f);
        REQUIRE(t.height_at(15, 15) == Approx(0.0f));
        REQUIRE(t.dirty());
    }

    SECTION("lower modifies cells within radius") {
        t.lower(10, 10, 0.5f, 2);
        REQUIRE(t.height_at(10, 10) < 0.0f);
    }

    SECTION("flatten brings cells toward target") {
        t.set_height(10, 10, 5.0f);
        t.set_height(9, 10, 1.0f);
        t.flatten(10, 10, 2.0f, 1);
        REQUIRE(t.height_at(10, 10) < 5.0f);
        REQUIRE(t.height_at(9, 10) > 1.0f);
    }
}

TEST_CASE("Terrain height_at_world interpolation", "[terrain]") {
    robcraft::engine::world::Terrain t(10, 10, 1.0);
    t.set_height(5, 5, 2.0f);
    t.set_height(6, 5, 4.0f);
    t.set_height(5, 6, 3.0f);
    t.set_height(6, 6, 5.0f);

    double half = 10.0 * 1.0 * 0.5;
    float h = t.height_at_world(5.5 * 1.0 - half, 5.5 * 1.0 - half);
    REQUIRE(h > 2.0f);
    REQUIRE(h < 5.0f);
}

TEST_CASE("Terrain save and load roundtrip", "[terrain]") {
    const std::string path = "/tmp/test_terrain.bin";

    {
        robcraft::engine::world::Terrain t(16, 16, 2.0);
        t.set_height(3, 3, 5.0f);
        t.set_walkable(3, 4, false);
        t.set_water(5, 5, true);
        REQUIRE(save_terrain(t, path));
    }

    robcraft::engine::world::Terrain loaded;
    REQUIRE(load_terrain(loaded, path));
    REQUIRE(loaded.width() == 16);
    REQUIRE(loaded.height() == 16);
    REQUIRE(loaded.cell_size() == Approx(2.0));
    REQUIRE(loaded.height_at(3, 3) == Approx(5.0f));
    REQUIRE(!loaded.is_walkable(3, 4));
    REQUIRE(loaded.has_water(5, 5));
    REQUIRE(loaded.water_surface_height(5, 5) ==
            Approx(robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));

    std::remove(path.c_str());
}

TEST_CASE("Terrain raycast hits ground", "[terrain]") {
    robcraft::engine::world::Terrain t(10, 10, 1.0);
    t.set_height(5, 5, 3.0f);

    double half = 10.0 * 1.0 * 0.5;
    double cx = 5.0 * 1.0 - half + 0.5;
    double cz = 5.0 * 1.0 - half + 0.5;

    robcraft::engine::collision::Ray ray;
    ray.origin = robcraft::engine::math::Vec3(cx, 10.0, cz);
    ray.direction = robcraft::engine::math::Vec3(0.0, -1.0, 0.0);

    auto hit = t.raycast(ray);
    REQUIRE(hit.has_value());
    REQUIRE(*hit > 0.0);
    REQUIRE(*hit < 12.0);
}

TEST_CASE("Terrain cliff_to_level", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 1.0);

    SECTION("level zero flattens") {
        t.set_height(3, 3, 5.0f);
        t.cliff_to_level(3, 3, 0, 1);
        REQUIRE(t.cliff_level(3, 3) == 0);
        REQUIRE(t.height_at(3, 3) == Approx(0.0f));
    }

    SECTION("height is level times step") {
        t.cliff_to_level(3, 3, 8, 1);
        REQUIRE(t.cliff_level(3, 3) == 8);
        REQUIRE(t.height_at(3, 3) == Approx(16.0f));
    }

    SECTION("level clamps at 16") {
        t.cliff_to_level(3, 3, 255, 1);
        REQUIRE(t.cliff_level(3, 3) == 16);
        REQUIRE(t.height_at(3, 3) == Approx(32.0f));
    }
}

TEST_CASE("Terrain generates mesh data", "[terrain]") {
    robcraft::engine::world::Terrain t(5, 5, 1.0);
    t.set_height(2, 2, 1.5f);

    auto data = build_terrain_mesh(t);
    REQUIRE(data.vertices.size() == 25);
    REQUIRE(data.indices.size() > 0);
}

TEST_CASE("Terrain mesh covers world bounds", "[terrain]") {
    robcraft::engine::world::Terrain t(64, 64, 1.0);
    auto data = build_terrain_mesh(t);
    REQUIRE(data.vertices.size() >= 64 * 64);

    // Check bounds: vertices should span from -32 to +32
    float min_x = 1e9, max_x = -1e9, min_z = 1e9, max_z = -1e9;
    for (auto& v : data.vertices) {
        min_x = std::min(min_x, v.x);
        max_x = std::max(max_x, v.x);
        min_z = std::min(min_z, v.z);
        max_z = std::max(max_z, v.z);
    }
    REQUIRE(min_x <= -30.0f);
    REQUIRE(max_x >= 30.0f);
    REQUIRE(min_z <= -30.0f);
    REQUIRE(max_z >= 30.0f);
}

TEST_CASE("Terrain grid data has no diagonals", "[terrain]") {
    robcraft::engine::world::Terrain t(4, 4, 2.0);
    t.set_height(2, 2, 5.0f);
    auto d = build_terrain_grid(t);
    // verticals: 5 lines x 4 segments x 2 verts; horizontals: 5 x 4 x 2
    REQUIRE(d.vertices.size() == 80);
    REQUIRE(d.indices.size() == 80);
    bool axis_aligned = true;
    for (size_t i = 0; i + 1 < d.vertices.size(); i += 2) {
        if (d.vertices[i].x != d.vertices[i + 1].x && d.vertices[i].z != d.vertices[i + 1].z)
            axis_aligned = false;
    }
    REQUIRE(axis_aligned);
    int raised = 0;
    for (auto& v : d.vertices)
        if (v.y == Approx(5.0f)) raised++;
    REQUIRE(raised == 4);  // cell (2,2) is a shared corner of 4 segments
}

TEST_CASE("Terrain per-cell water", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 1.0);
    t.set_height(3, 4, 2.0f);

    SECTION("default is dry") {
        REQUIRE(!t.has_water(3, 3));
        REQUIRE(t.water_surface_height(3, 3) < -500.0f);
    }

    SECTION("set and query water") {
        t.set_water(3, 4, true);
        REQUIRE(t.has_water(3, 4));
        REQUIRE(t.water_surface_height(3, 4) ==
                Approx(2.0f + robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));
        REQUIRE(!t.has_water(2, 4));
        REQUIRE(t.dirty());
    }

    SECTION("surface tracks terrain height") {
        t.set_water(3, 4, true);
        float s1 = t.water_surface_height(3, 4);
        t.set_height(3, 4, 5.0f);
        REQUIRE(t.water_surface_height(3, 4) == Approx(s1 + 3.0f));
    }

    SECTION("clear sets dry") {
        t.set_water(3, 4, true);
        t.set_water(3, 4, false);
        REQUIRE(!t.has_water(3, 4));
    }

    SECTION("paint water in circular brush") {
        t.paint_water(4, 4, true, 1);
        REQUIRE(t.has_water(4, 4));
        REQUIRE(t.has_water(3, 4));
        REQUIRE(t.has_water(5, 4));
        REQUIRE(!t.has_water(5, 5));
        REQUIRE(!t.has_water(1, 1));
    }
}

TEST_CASE("Water mesh UVs encode shore distance and terrain height", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 1.0);
    for (int z = 0; z < 8; ++z)
        for (int x = 0; x < 8; ++x) t.set_height(x, z, 2.0f);
    // Solid 3x3 water block over cells 1..3 so cell (2,2) is fully surrounded
    // (interior, shore distance 2) while cell (3,2) touches dry land (shore 1).
    for (int z = 1; z <= 3; ++z)
        for (int x = 1; x <= 3; ++x) t.set_water(x, z, true);
    auto d = build_terrain_water_mesh(t);

    REQUIRE_FALSE(d.vertices.empty());

    // The loop processes cells z-outer/x-inner, so the first vertex at the shared
    // (3,3) corner belongs to cell (2,2), the interior (shore=2) one. find_corner
    // returns the first matching vertex, which relies on that iteration order.
    auto find_corner = [&](int cx, int cz) {
        double half = t.width() * t.cell_size() * 0.5;
        double x = cx * t.cell_size() - half;
        double z = cz * t.cell_size() - half;
        for (const auto& v : d.vertices)
            if (std::abs(v.x - (float)x) < 1e-4f && std::abs(v.z - (float)z) < 1e-4f) return v;
        return robcraft::renderer::Vertex{};
    };

    auto inner = find_corner(3, 3);
    REQUIRE(inner.v == Approx(2.0f));
    REQUIRE(inner.u > 0.9f);

    auto outer = find_corner(4, 3);
    REQUIRE(outer.u < inner.u);
}

TEST_CASE("Water mesh follows terrain slope", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 1.0);
    // Ramp: height increases along +X, so each cell's corners sit at different heights.
    for (int z = 0; z < 8; ++z)
        for (int x = 0; x < 8; ++x) t.set_height(x, z, static_cast<float>(x));
    t.paint_water(2, 2, true, 1);
    auto d = build_terrain_water_mesh(t);
    REQUIRE_FALSE(d.vertices.empty());

    double half = t.width() * t.cell_size() * 0.5;
    auto find_corner = [&](int cx, int cz) {
        double x = cx * t.cell_size() - half;
        double z = cz * t.cell_size() - half;
        for (const auto& v : d.vertices)
            if (std::abs(v.x - (float)x) < 1e-4f && std::abs(v.z - (float)z) < 1e-4f) return v;
        return robcraft::renderer::Vertex{};
    };

    // Cell (2,2) corners: terrain heights 2,3,3,2 (x/z order) + WATER_SURFACE_OFFSET.
    auto c00 = find_corner(2, 2);
    auto c10 = find_corner(3, 2);
    auto c11 = find_corner(3, 3);
    auto c01 = find_corner(2, 3);
    REQUIRE(c00.y == Approx(2.0f + robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));
    REQUIRE(c10.y == Approx(3.0f + robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));
    REQUIRE(c11.y == Approx(3.0f + robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));
    REQUIRE(c01.y == Approx(2.0f + robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));
}

TEST_CASE("Water triangulation matches terrain fold", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 1.0);
    // Fold: cell (2,2) with corner heights 2,3,3,2 (x/z order) — the same ramp as
    // the slope test, but here we verify the quad is split along the terrain mesh's
    // diagonal so the two surfaces don't tear apart.
    for (int z = 0; z < 8; ++z)
        for (int x = 0; x < 8; ++x) t.set_height(x, z, static_cast<float>(x));
    t.set_water(2, 2, true);
    auto d = build_terrain_water_mesh(t);
    REQUIRE_FALSE(d.vertices.empty());

    // The water quad for cell (2,2) has corners at grid positions (2,2),(3,2),(3,3),(2,3)
    // in the same order the terrain mesh uses: a=(x,z), b=(x+1,z), c=(x,z+1), d=(x+1,z+1).
    // Terrain triangles are (a,c,b),(b,c,d) — diagonal b-c. The water quad must use the
    // same diagonal so terrain never pokes through: triangles (a,c,b),(b,c,d) too.
    //
    // Corners in world space:
    double half = t.width() * t.cell_size() * 0.5;
    auto pos = [&](int cx, int cz) {
        return robcraft::engine::math::Vec3(cx * t.cell_size() - half, 0,
                                            cz * t.cell_size() - half);
    };
    auto v_a = pos(2, 2), v_b = pos(3, 2), v_c = pos(2, 3), v_d = pos(3, 3);

    // Find indices of the water vertices at those corners.
    auto find_idx = [&](const robcraft::engine::math::Vec3& p) {
        for (size_t i = 0; i < d.vertices.size(); ++i) {
            const auto& v = d.vertices[i];
            if (std::abs(v.x - (float)p.x) < 1e-4f && std::abs(v.z - (float)p.z) < 1e-4f)
                return static_cast<unsigned int>(i);
        }
        return static_cast<unsigned int>(d.vertices.size());
    };
    unsigned int ia = find_idx(v_a), ib = find_idx(v_b), ic = find_idx(v_c), id = find_idx(v_d);
    REQUIRE(ia != d.vertices.size());
    REQUIRE(ib != d.vertices.size());
    REQUIRE(ic != d.vertices.size());
    REQUIRE(id != d.vertices.size());

    // Terrain diagonal connects b (x+1,z) and c (x,z+1); water must share it.
    // Check both triangle winding orders are consistent with (a,c,b),(b,c,d).
    bool matches = false;
    for (size_t i = 0; i + 2 < d.indices.size() && !matches; i += 3) {
        unsigned int t0 = d.indices[i], t1 = d.indices[i + 1], t2 = d.indices[i + 2];
        matches = (t0 == ia && t1 == ic && t2 == ib) || (t0 == ib && t1 == ic && t2 == id);
    }
    REQUIRE(matches);
}

TEST_CASE("Terrain load old format without water array", "[terrain]") {
    const std::string path = "/tmp/test_terrain_old.bin";
    {
        // Write the pre-water format: w,h,cs,heights,walkable,cliff,type,water_level(double)
        std::ofstream f(path, std::ios::binary);
        int32_t w = 4, h = 4;
        float cs = 1.0f;
        double water_level = 2.0;
        std::vector<float> heights(16, 0.0f);
        std::vector<uint8_t> bytes(16, 1);
        bytes[0] = 0;  // cell (0,0) non-walkable
        f.write(reinterpret_cast<const char*>(&w), 4);
        f.write(reinterpret_cast<const char*>(&h), 4);
        f.write(reinterpret_cast<const char*>(&cs), 4);
        f.write(reinterpret_cast<const char*>(heights.data()), 16 * 4);
        f.write(reinterpret_cast<const char*>(bytes.data()), 16);
        f.write(reinterpret_cast<const char*>(bytes.data()), 16);
        f.write(reinterpret_cast<const char*>(bytes.data()), 16);
        f.write(reinterpret_cast<const char*>(&water_level), 8);
    }

    robcraft::engine::world::Terrain loaded;
    REQUIRE(load_terrain(loaded, path));
    REQUIRE(loaded.width() == 4);
    REQUIRE(loaded.height() == 4);
    REQUIRE(!loaded.is_walkable(0, 0));
    REQUIRE(!loaded.has_water(2, 2));  // old files have no per-cell water -> dry

    std::remove(path.c_str());
}

TEST_CASE("Terrain load old format reinterprets absolute water heights as presence", "[terrain]") {
    const std::string path = "/tmp/test_terrain_old_water.bin";
    {
        // Write the legacy format: w,h,cs,heights,walkable,cliff,type,water(float[]).
        // Legacy water slots held absolute heights; any value > WATER_OFF means wet.
        std::ofstream f(path, std::ios::binary);
        int32_t w = 4, h = 4;
        float cs = 1.0f;
        std::vector<float> heights(16, 0.0f);
        heights[1 * 4 + 1] = 1.0f;  // cell (1,1) height
        std::vector<uint8_t> bytes(16, 1);
        std::vector<float> water(16, robcraft::engine::world::Terrain::WATER_OFF);
        water[1 * 4 + 1] = 2.5f;  // legacy absolute water height at cell (1,1)
        f.write(reinterpret_cast<const char*>(&w), 4);
        f.write(reinterpret_cast<const char*>(&h), 4);
        f.write(reinterpret_cast<const char*>(&cs), 4);
        f.write(reinterpret_cast<const char*>(heights.data()), 16 * 4);
        f.write(reinterpret_cast<const char*>(bytes.data()), 16);
        f.write(reinterpret_cast<const char*>(bytes.data()), 16);
        f.write(reinterpret_cast<const char*>(bytes.data()), 16);
        f.write(reinterpret_cast<const char*>(water.data()), 16 * 4);
    }

    robcraft::engine::world::Terrain loaded;
    REQUIRE(load_terrain(loaded, path));
    REQUIRE(loaded.width() == 4);
    REQUIRE(loaded.height() == 4);
    REQUIRE(loaded.has_water(1, 1));
    REQUIRE(loaded.water_surface_height(1, 1) ==
            Approx(1.0f + robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));
    REQUIRE(!loaded.has_water(0, 0));  // WATER_OFF cell stays dry

    std::remove(path.c_str());
}

TEST_CASE("Terrain splat weights blend at type boundaries", "[terrain]") {
    robcraft::engine::world::Terrain t(4, 4, 2.0);
    t.set_terrain_type(0, 0, robcraft::engine::world::TerrainType::Grass);
    t.set_terrain_type(0, 1, robcraft::engine::world::TerrainType::Dirt);
    // Vertex at corner (1,1) touches cells (1,1), (0,1), (1,0), (0,0).
    // World x = 1*2 - half(4) = -2, z likewise.
    auto data = build_terrain_mesh(t);
    REQUIRE(data.weights.size() == data.vertices.size() * 4);

    // Find the vertex at world (-2, -2)
    size_t idx = 0;
    bool found = false;
    for (size_t i = 0; i < data.vertices.size(); ++i) {
        if (data.vertices[i].x == -2.0f && data.vertices[i].z == -2.0f) {
            idx = i;
            found = true;
            break;
        }
    }
    REQUIRE(found);
    // Neighbor cells: (1,1)=Grass default, (0,1)=Dirt, (1,0)=Grass default, (0,0)=Grass.
    // weights order: [Grass, Dirt, Rock, Sand]
    const float* w = &data.weights[idx * 4];
    float sum = w[0] + w[1] + w[2] + w[3];
    REQUIRE(sum == Approx(1.0f).margin(1e-4));
    REQUIRE(w[0] > 0.0f);           // grass present (3 of 4 cells)
    REQUIRE(w[1] > 0.0f);           // dirt present (1 of 4 cells)
    REQUIRE(w[2] == Approx(0.0f));  // rock absent
    REQUIRE(w[3] == Approx(0.0f));  // sand absent
}

TEST_CASE("Terrain splat weights are all-zero for pure snow (implied 5th layer)", "[terrain]") {
    robcraft::engine::world::Terrain t(4, 4, 2.0);
    for (int x = 0; x < 4; ++x)
        for (int z = 0; z < 4; ++z)
            t.set_terrain_type(x, z, robcraft::engine::world::TerrainType::Snow);
    auto data = build_terrain_mesh(t);
    REQUIRE(data.weights.size() == data.vertices.size() * 4);
    bool found = false;
    for (size_t i = 0; i < data.vertices.size(); ++i) {
        if (data.vertices[i].x == -2.0f && data.vertices[i].z == -2.0f) {
            const float* w = &data.weights[i * 4];
            REQUIRE(w[0] == Approx(0.0f));
            REQUIRE(w[1] == Approx(0.0f));
            REQUIRE(w[2] == Approx(0.0f));
            REQUIRE(w[3] == Approx(0.0f));
            found = true;
            break;
        }
    }
    REQUIRE(found);
}

TEST_CASE("Terrain splat weights stay in sync with vertices on cliffed terrain", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 2.0);
    // Raise one region to a different cliff level.
    for (int x = 0; x < 4; ++x)
        for (int z = 0; z < 4; ++z) t.set_cliff_level(x, z, 1);
    auto data = build_terrain_mesh(t);
    REQUIRE(data.vertices.size() == 8 * 8);  // plain grid, no wall geometry
    REQUIRE(data.weights.size() == data.vertices.size() * 4);
    for (float w : data.weights) {
        REQUIRE(w >= 0.0f);
        REQUIRE(w <= 1.0f);
    }
}

TEST_CASE("Cliff tool blends a smooth elevation like the raise tool", "[terrain]") {
    robcraft::engine::world::Terrain t(20, 20, 1.0);
    t.cliff_to_level(10, 10, 4, 3);
    // The center reaches the target level (level 4 = 8.0).
    REQUIRE(t.height_at(10, 10) == Approx(8.0f));
    REQUIRE(t.cliff_level(10, 10) == 4);
    // Heights fall off gradually toward the brush edge...
    REQUIRE(t.height_at(11, 10) < t.height_at(10, 10));
    REQUIRE(t.height_at(12, 10) < t.height_at(11, 10));
    REQUIRE(t.height_at(13, 10) < t.height_at(12, 10));
    // ...and blend into untouched terrain beyond the brush.
    REQUIRE(t.height_at(14, 10) == Approx(0.0f));
}

TEST_CASE("Cliff elevations render as a smooth heightfield without walls", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 2.0);
    t.cliff_to_level(4, 4, 4, 2);
    auto data = build_terrain_mesh(t);
    // Plain grid: one vertex per corner, no added wall/block geometry.
    REQUIRE(data.vertices.size() == 8 * 8);
    for (auto& v : data.vertices) REQUIRE(v.ny > 0.0f);  // no vertical faces
}

TEST_CASE("Terrain raycast follows the smooth cliff elevation", "[terrain]") {
    robcraft::engine::world::Terrain t(8, 8, 2.0);
    t.cliff_to_level(4, 4, 4, 2);
    auto cc = t.cell_center_world(4, 4);
    robcraft::engine::collision::Ray ray{robcraft::engine::math::Vec3(cc.x, 20.0, cc.z),
                                         robcraft::engine::math::Vec3(0, -1, 0)};
    auto hit = t.raycast(ray);
    REQUIRE(hit.has_value());
    robcraft::engine::math::Vec3 p = ray.origin + ray.direction * *hit;
    REQUIRE(p.y == Approx(t.height_at_world(cc.x, cc.z)).margin(0.01));
}

TEST_CASE("Terrain raycast hits a lowered pit floor", "[terrain]") {
    robcraft::engine::world::Terrain t(64, 64, 2.0);
    // Mimic the editor's lower tool: repeated brush lowers dig a shallow pit.
    int cx = 32, cz = 32;
    for (int i = 0; i < 5; ++i) t.lower(cx, cz, 0.5f, 4);
    REQUIRE(t.height_at(cx, cz) < -2.0f);

    // Aim from the editor camera at the pit floor; this ray clears the rims.
    auto cc = t.cell_center_world(cx, cz);
    robcraft::engine::math::Vec3 origin(50.0, 40.0, 50.0);
    robcraft::engine::math::Vec3 dir = (cc - origin).normalized();
    auto hit = t.raycast({origin, dir});
    REQUIRE(hit.has_value());
    robcraft::engine::math::Vec3 p = origin + dir * *hit;
    int hcx, hcz;
    t.world_to_cell(p.x, p.z, hcx, hcz);
    REQUIRE(hcx == cx);
    REQUIRE(hcz == cz);
}

TEST_CASE("Terrain raycast survives a single lower step", "[terrain]") {
    robcraft::engine::world::Terrain t(64, 64, 2.0);
    t.lower(32, 32, 0.5f, 4);
    REQUIRE(t.height_at(32, 32) < 0.0f);

    auto cc = t.cell_center_world(32, 32);
    robcraft::engine::math::Vec3 origin(50.0, 40.0, 50.0);
    robcraft::engine::math::Vec3 dir = (cc - origin).normalized();
    auto hit = t.raycast({origin, dir});
    REQUIRE(hit.has_value());
    robcraft::engine::math::Vec3 p = origin + dir * *hit;
    int hcx, hcz;
    t.world_to_cell(p.x, p.z, hcx, hcz);
    REQUIRE(hcx == 32);
    REQUIRE(hcz == 32);
}

TEST_CASE("Terrain raycast hits terrain from outside the footprint", "[terrain]") {
    robcraft::engine::world::Terrain t(32, 32, 2.0);        // footprint x/z in [-32, 32]
    robcraft::engine::math::Vec3 origin(38.0, 30.0, 38.0);  // outside the footprint
    robcraft::engine::math::Vec3 dir =
        (robcraft::engine::math::Vec3(0.0, 0.0, 0.0) - origin).normalized();
    auto hit = t.raycast({origin, dir});
    REQUIRE(hit.has_value());
    robcraft::engine::math::Vec3 p = origin + dir * *hit;
    REQUIRE(p.y == Catch::Approx(0.0).margin(0.01));
}

TEST_CASE("Terrain vertical ray from outside footprint misses", "[terrain]") {
    robcraft::engine::world::Terrain t(32, 32, 2.0);
    robcraft::engine::collision::Ray ray{robcraft::engine::math::Vec3(50.0, 40.0, 0.0),
                                         robcraft::engine::math::Vec3(0.0, -1.0, 0.0)};
    REQUIRE(!t.raycast(ray).has_value());
}

TEST_CASE("Terrain water plane height is the mean surface height", "[terrain]") {
    robcraft::engine::world::Terrain t(4, 4, 2.0);

    SECTION("no water returns WATER_OFF") {
        REQUIRE(t.water_plane_height() == Approx(robcraft::engine::world::Terrain::WATER_OFF));
    }

    SECTION("single wet cell at default height") {
        t.set_water(1, 1, true);
        REQUIRE(t.water_plane_height() ==
                Approx(robcraft::engine::world::Terrain::WATER_SURFACE_OFFSET));
    }

    SECTION("mean over cells with different terrain heights") {
        t.set_water(1, 1, true);
        t.set_water(2, 2, true);
        t.set_height(2, 2, 3.0f);
        // (0.0 + 0.5) and (3.0 + 0.5) -> mean 2.0
        REQUIRE(t.water_plane_height() == Approx(2.0f));
    }
}

TEST_CASE("Terrain raycast rejects grazing and outward-pointing rays", "[terrain]") {
    robcraft::engine::world::Terrain t(32, 32, 2.0);

    SECTION("grazing the footprint boundary misses") {
        robcraft::engine::collision::Ray ray{robcraft::engine::math::Vec3(32.0, 10.0, 0.0),
                                             robcraft::engine::math::Vec3(1.0, -1.0, 0.0)};
        REQUIRE(!t.raycast(ray).has_value());
    }

    SECTION("pointing away from the footprint misses") {
        robcraft::engine::math::Vec3 origin(38.0, 30.0, 38.0);
        robcraft::engine::math::Vec3 dir =
            robcraft::engine::math::Vec3(1.0, -1.0, 1.0).normalized();
        REQUIRE(!t.raycast({origin, dir}).has_value());
    }
}
