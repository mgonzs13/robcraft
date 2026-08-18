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

#include "robcraft/engine/world/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robcraft::engine::world {

using namespace robcraft::engine::world;

Terrain::Terrain() {}

Terrain::Terrain(int width, int height, double cell_size)
    : width_(width), height_(height), cell_size_(cell_size) {
    size_t count = static_cast<size_t>(this->width_ * this->height_);
    this->heights_.resize(count, 0.0f);
    this->walkable_.resize(count, 1);
    this->cliff_level_.resize(count, 0);
    this->terrain_type_.resize(count, static_cast<uint8_t>(TerrainType::Grass));
    this->water_.resize(count, Terrain::WATER_OFF);
}

bool Terrain::in_bounds(int cx, int cz) const {
    return cx >= 0 && cx < this->width_ && cz >= 0 && cz < this->height_;
}

float Terrain::height_at(int cx, int cz) const {
    if (!this->in_bounds(cx, cz)) return 0.0f;
    return this->heights_[this->index(cx, cz)];
}

float Terrain::height_at_world(double wx, double wz) const {
    double half = this->width_ * this->cell_size_ * 0.5;
    double fx = (wx + half) / this->cell_size_;
    double fz = (wz + half) / this->cell_size_;
    int cx = static_cast<int>(std::floor(fx));
    int cz = static_cast<int>(std::floor(fz));
    double tx = fx - cx;
    double tz = fz - cz;

    if (tx < 0) {
        cx--;
        tx += 1.0;
    }
    if (tz < 0) {
        cz--;
        tz += 1.0;
    }
    if (!this->in_bounds(cx, cz) || !this->in_bounds(cx + 1, cz + 1)) return 0.0f;

    float h00 = this->heights_[this->index(cx, cz)];
    float h10 = this->heights_[this->index(cx + 1, cz)];
    float h01 = this->heights_[this->index(cx, cz + 1)];
    float h11 = this->heights_[this->index(cx + 1, cz + 1)];

    float h0 = h00 + static_cast<float>(tx) * (h10 - h00);
    float h1 = h01 + static_cast<float>(tx) * (h11 - h01);
    return h0 + static_cast<float>(tz) * (h1 - h0);
}

void Terrain::set_height(int cx, int cz, float h) {
    if (!this->in_bounds(cx, cz)) return;
    this->heights_[this->index(cx, cz)] = h;
    this->mark_dirty();
}

bool Terrain::is_walkable(int cx, int cz) const {
    if (!this->in_bounds(cx, cz)) return false;
    return this->walkable_[this->index(cx, cz)] != 0;
}

void Terrain::set_walkable(int cx, int cz, bool w) {
    if (!this->in_bounds(cx, cz)) return;
    this->walkable_[this->index(cx, cz)] = w ? 1 : 0;
}

std::optional<double> Terrain::raycast(const Ray& ray) const {
    double ox = ray.origin.x;
    double oy = ray.origin.y;
    double oz = ray.origin.z;
    double dx = ray.direction.x;
    double dy = ray.direction.y;
    double dz = ray.direction.z;

    if (dy >= 0) return std::nullopt;

    double half = this->width_ * this->cell_size_ * 0.5;
    double lo = -half;
    double hi = half;

    // Vertical ray: only hits if its x/z position is inside the footprint.
    if (std::abs(dx) < 1e-10 && std::abs(dz) < 1e-10) {
        if (ox < lo || ox > hi || oz < lo || oz > hi) return std::nullopt;
        double h = this->height_at_world(ox, oz);
        if (oy >= h) return (oy - static_cast<double>(h)) / (-dy);
        return std::nullopt;
    }

    // Clip the ray to the footprint rectangle (slab method) to find the entry t.
    double t_enter = 0.0;
    double t_exit = 1e18;
    auto clip_axis = [&](double o, double d, double lo_a, double hi_a) -> bool {
        if (std::abs(d) < 1e-10) return o >= lo_a && o <= hi_a;
        double t1 = (lo_a - o) / d;
        double t2 = (hi_a - o) / d;
        if (t1 > t2) std::swap(t1, t2);
        t_enter = std::max(t_enter, t1);
        t_exit = std::min(t_exit, t2);
        return t_enter <= t_exit;
    };
    if (!clip_axis(ox, dx, lo, hi)) return std::nullopt;
    if (!clip_axis(oz, dz, lo, hi)) return std::nullopt;
    if (t_enter >= t_exit) return std::nullopt;
    t_enter = std::max(t_enter, 0.0);

    // Entry cell (clamped into the DDA's valid range).
    double ex = ox + dx * t_enter;
    double ez = oz + dz * t_enter;
    int cx = static_cast<int>(std::floor((ex + half) / this->cell_size_));
    int cz = static_cast<int>(std::floor((ez + half) / this->cell_size_));
    cx = std::clamp(cx, 0, this->width_ - 2);
    cz = std::clamp(cz, 0, this->height_ - 2);

    int step_x = (dx > 0) ? 1 : -1;
    int step_z = (dz > 0) ? 1 : -1;
    double t_delta_x = (std::abs(dx) > 1e-10) ? std::abs(this->cell_size_ / dx)
                                              : std::numeric_limits<double>::infinity();
    double t_delta_z = (std::abs(dz) > 1e-10) ? std::abs(this->cell_size_ / dz)
                                              : std::numeric_limits<double>::infinity();

    double next_x =
        (dx > 0) ? ((cx + 1) * this->cell_size_ - half) : (cx * this->cell_size_ - half);
    double next_z =
        (dz > 0) ? ((cz + 1) * this->cell_size_ - half) : (cz * this->cell_size_ - half);
    double t_max_x =
        (std::abs(dx) > 1e-10) ? (next_x - ox) / dx : std::numeric_limits<double>::infinity();
    double t_max_z =
        (std::abs(dz) > 1e-10) ? (next_z - oz) / dz : std::numeric_limits<double>::infinity();

    // Ray-triangle intersection (Möller–Trumbore); returns -1 on miss.
    auto tri_hit = [&](const Vec3& p0, const Vec3& p1, const Vec3& p2) -> double {
        Vec3 e1 = p1 - p0;
        Vec3 e2 = p2 - p0;
        Vec3 pv = ray.direction.cross(e2);
        double det = e1.dot(pv);
        if (std::abs(det) < 1e-12) return -1.0;
        double inv = 1.0 / det;
        Vec3 tv = ray.origin - p0;
        double u = tv.dot(pv) * inv;
        if (u < 0.0 || u > 1.0) return -1.0;
        Vec3 qv = tv.cross(e1);
        double v = ray.direction.dot(qv) * inv;
        if (v < 0.0 || u + v > 1.0) return -1.0;
        double t = e2.dot(qv) * inv;
        return (t >= 0.0) ? t : -1.0;
    };

    int iters = 0;
    const int max_iters = this->width_ + this->height_ + 10;
    double t = t_enter;
    double best_t = -1.0;

    while (iters++ < max_iters && t < 1000.0) {
        if (cx < 0 || cx >= this->width_ - 1 || cz < 0 || cz >= this->height_ - 1) break;

        // Test the two triangles that make up this cell's surface; this exactly
        // matches generate_mesh_data()'s triangulation so hits agree with the
        // rendered mesh even on concave (lowered) terrain.
        double x0 = cx * this->cell_size_ - half;
        double z0 = cz * this->cell_size_ - half;
        double x1 = x0 + this->cell_size_;
        double z1 = z0 + this->cell_size_;
        Vec3 p00(x0, this->height_at_vert(cx, cz), z0);
        Vec3 p10(x1, this->height_at_vert(cx + 1, cz), z0);
        Vec3 p01(x0, this->height_at_vert(cx, cz + 1), z1);
        Vec3 p11(x1, this->height_at_vert(cx + 1, cz + 1), z1);
        for (double th : {tri_hit(p00, p01, p10), tri_hit(p10, p01, p11)}) {
            if (th >= 0.0 && th >= t - 1e-9 && (best_t < 0.0 || th < best_t)) best_t = th;
        }

        if (t_max_x < t_max_z) {
            t = t_max_x;
            cx += step_x;
            t_max_x += t_delta_x;
        } else {
            t = t_max_z;
            cz += step_z;
            t_max_z += t_delta_z;
        }
    }

    if (best_t >= 0.0) return best_t;
    return std::nullopt;
}

Vec3 Terrain::cell_center_world(int cx, int cz) const {
    double half = this->width_ * this->cell_size_ * 0.5;
    double x = cx * this->cell_size_ + this->cell_size_ * 0.5 - half;
    double z = cz * this->cell_size_ + this->cell_size_ * 0.5 - half;
    float y = this->height_at(cx, cz);
    return Vec3(x, static_cast<double>(y), z);
}

void Terrain::world_to_cell(double wx, double wz, int& cx, int& cz) const {
    double half = this->width_ * this->cell_size_ * 0.5;
    cx = static_cast<int>(std::floor((wx + half) / this->cell_size_));
    cz = static_cast<int>(std::floor((wz + half) / this->cell_size_));
}

void Terrain::raise(int cx, int cz, float amount, int brush_radius) {
    this->for_each_brush_cell(cx, cz, brush_radius, [&](int c, int r, float dist) {
        float falloff = 1.0f - dist / (brush_radius + 1.0f);
        this->heights_[this->index(c, r)] += amount * falloff;
    });
    this->mark_dirty();
}

void Terrain::lower(int cx, int cz, float amount, int brush_radius) {
    this->raise(cx, cz, -amount, brush_radius);
}

void Terrain::flatten(int cx, int cz, float target, int brush_radius) {
    this->for_each_brush_cell(cx, cz, brush_radius, [&](int c, int r, float dist) {
        float falloff = 1.0f - dist / (brush_radius + 1.0f);
        float& h = this->heights_[this->index(c, r)];
        h = h + (target - h) * falloff * 0.5f;
    });
    this->mark_dirty();
}

uint8_t Terrain::cliff_level(int cx, int cz) const {
    if (!this->in_bounds(cx, cz)) return 0;
    return this->cliff_level_[this->index(cx, cz)];
}

void Terrain::set_cliff_level(int cx, int cz, uint8_t level) {
    if (!this->in_bounds(cx, cz)) return;
    this->cliff_level_[this->index(cx, cz)] = level;
    this->mark_dirty();
}

TerrainType Terrain::terrain_type(int cx, int cz) const {
    if (!this->in_bounds(cx, cz)) return TerrainType::Grass;
    return static_cast<TerrainType>(this->terrain_type_[this->index(cx, cz)]);
}

void Terrain::set_terrain_type(int cx, int cz, TerrainType type) {
    if (!this->in_bounds(cx, cz)) return;
    this->terrain_type_[this->index(cx, cz)] = static_cast<uint8_t>(type);
    this->mark_dirty();
}

void Terrain::paint_type(int cx, int cz, TerrainType type, int brush_radius) {
    this->for_each_brush_cell(cx, cz, brush_radius, [&](int c, int r, float) {
        this->terrain_type_[this->index(c, r)] = static_cast<uint8_t>(type);
    });
    this->mark_dirty();
}

void Terrain::cliff_to_level(int cx, int cz, uint8_t level, int brush_radius) {
    uint8_t target = std::min(level, Terrain::MAX_CLIFF_LEVEL);
    float target_h = static_cast<float>(target) * Terrain::CLIFF_HEIGHT;
    this->for_each_brush_cell(cx, cz, brush_radius, [&](int c, int r, float dist) {
        // Smooth falloff like the raise tool, so the elevation blends into
        // the surrounding terrain instead of forming a hard block.
        float falloff = 1.0f - dist / (brush_radius + 1.0f);
        float& h = this->heights_[this->index(c, r)];
        h = h + (target_h - h) * falloff;
        if (std::abs(h - target_h) < Terrain::CLIFF_HEIGHT * 0.5f)
            this->cliff_level_[this->index(c, r)] = target;
    });
    this->mark_dirty();
}

bool Terrain::has_water(int cx, int cz) const {
    if (!this->in_bounds(cx, cz)) return false;
    return this->water_[this->index(cx, cz)] > Terrain::WATER_OFF;
}

float Terrain::water_surface_height(int cx, int cz) const {
    if (!this->has_water(cx, cz)) return Terrain::WATER_OFF;
    return this->height_at(cx, cz) + Terrain::WATER_SURFACE_OFFSET;
}

void Terrain::set_water(int cx, int cz, bool present) {
    if (!this->in_bounds(cx, cz)) return;
    this->water_[this->index(cx, cz)] = present ? 0.0f : Terrain::WATER_OFF;
    this->mark_dirty();
}

void Terrain::paint_water(int cx, int cz, bool present, int brush_radius) {
    float value = present ? 0.0f : Terrain::WATER_OFF;
    this->for_each_brush_cell(cx, cz, brush_radius, [&](int c, int r, float) {
        this->water_[this->index(c, r)] = value;
    });
    this->mark_dirty();
}

float Terrain::water_plane_height() const {
    double sum = 0.0;
    int count = 0;
    for (int z = 0; z < this->height_; ++z) {
        for (int x = 0; x < this->width_; ++x) {
            if (this->has_water(x, z)) {
                sum += this->water_surface_height(x, z);
                ++count;
            }
        }
    }
    return count == 0 ? Terrain::WATER_OFF : static_cast<float>(sum / count);
}

float Terrain::height_at_vert(int cx, int cz) const {
    if (cx < 0 || cx >= this->width_ || cz < 0 || cz >= this->height_) return 0.0f;
    return this->heights_[this->index(cx, cz)];
}

}  // namespace robcraft::engine::world
