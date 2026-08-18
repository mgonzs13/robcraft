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

#include "robcraft/renderer/mesh.hpp"

#include <cmath>

#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

namespace {

/**
 * @brief Appends a capped cylinder (ring + side quads + cap triangles).
 * @param verts Vertex buffer to append to.
 * @param idx Index buffer to append to.
 * @param seg Ring segment count.
 * @param radius Ring radius.
 * @param y0 Bottom height.
 * @param y1 Top height.
 * @param normal_scale Scale for the ring normal (1.0 for a unit circle,
 *        radius for the tree trunks that use the unnormalized ring vector).
 * @param top_col Top/side-top color (RGB).
 * @param bot_col Bottom/side-bottom color (RGB).
 */
void add_cylinder(std::vector<Vertex>& verts, std::vector<GLuint>& idx, int seg, float radius,
                  float y0, float y1, float normal_scale, const Vec3& top_col,
                  const Vec3& bot_col) {
    GLuint top_c = static_cast<GLuint>(verts.size());
    verts.push_back({0, y1, 0, 0, 1, 0, static_cast<float>(top_col.x),
                     static_cast<float>(top_col.y), static_cast<float>(top_col.z), 0.0f, 0.0f, 1.0f,
                     0.0f, 0.0f});
    GLuint bot_c = static_cast<GLuint>(verts.size());
    verts.push_back({0, y0, 0, 0, -1, 0, static_cast<float>(bot_col.x),
                     static_cast<float>(bot_col.y), static_cast<float>(bot_col.z), 0.0f, 0.0f, 1.0f,
                     0.0f, 0.0f});
    for (int i = 0; i <= seg; ++i) {
        float a = static_cast<float>(i) / seg * static_cast<float>(robcraft::engine::math::kTwoPi);
        float c = std::cos(a) * radius, s = std::sin(a) * radius;
        float nx = std::cos(a) * normal_scale, nz = std::sin(a) * normal_scale;
        verts.push_back({c, y1, s, nx, 0, nz, static_cast<float>(top_col.x),
                         static_cast<float>(top_col.y), static_cast<float>(top_col.z), 0.0f, 0.0f,
                         1.0f, 0.0f, 0.0f});
        verts.push_back({c, y0, s, nx, 0, nz, static_cast<float>(bot_col.x),
                         static_cast<float>(bot_col.y), static_cast<float>(bot_col.z), 0.0f, 0.0f,
                         1.0f, 0.0f, 0.0f});
    }
    for (int i = 0; i < seg; ++i) {
        GLuint t0 = top_c + 2 + i * 2, t1 = top_c + 2 + i * 2 + 2;
        GLuint b0 = bot_c + 2 + i * 2, b1 = bot_c + 2 + i * 2 + 2;
        // Side
        idx.push_back(t0);
        idx.push_back(b0);
        idx.push_back(t1);
        idx.push_back(t1);
        idx.push_back(b0);
        idx.push_back(b1);
        // Top cap
        idx.push_back(top_c);
        idx.push_back(t1);
        idx.push_back(t0);
        // Bottom cap
        idx.push_back(bot_c);
        idx.push_back(b0);
        idx.push_back(b1);
    }
}

}  // namespace

Mesh::~Mesh() {
    this->destroy();
}

Mesh::Mesh(Mesh&& o) noexcept
    : vao_(o.vao_),
      vbo_(o.vbo_),
      ebo_(o.ebo_),
      index_count_(o.index_count_),
      draw_mode_(o.draw_mode_),
      vbo_weights_(o.vbo_weights_),
      vbo_joint_weights_(o.vbo_joint_weights_),
      vbo_joint_indices_(o.vbo_joint_indices_) {
    o.vao_ = 0;
    o.vbo_ = 0;
    o.ebo_ = 0;
    o.index_count_ = 0;
    o.vbo_weights_ = 0;
    o.vbo_joint_weights_ = 0;
    o.vbo_joint_indices_ = 0;
}

Mesh& Mesh::operator=(Mesh&& o) noexcept {
    if (this != &o) {
        this->destroy();
        this->vao_ = o.vao_;
        this->vbo_ = o.vbo_;
        this->ebo_ = o.ebo_;
        this->index_count_ = o.index_count_;
        this->draw_mode_ = o.draw_mode_;
        this->vbo_weights_ = o.vbo_weights_;
        this->vbo_joint_weights_ = o.vbo_joint_weights_;
        this->vbo_joint_indices_ = o.vbo_joint_indices_;
        o.vao_ = 0;
        o.vbo_ = 0;
        o.ebo_ = 0;
        o.index_count_ = 0;
        o.vbo_weights_ = 0;
        o.vbo_joint_weights_ = 0;
        o.vbo_joint_indices_ = 0;
    }
    return *this;
}

void Mesh::destroy() {
    if (this->vbo_joint_weights_) glDeleteBuffers(1, &this->vbo_joint_weights_);
    if (this->vbo_joint_indices_) glDeleteBuffers(1, &this->vbo_joint_indices_);
    if (this->vbo_weights_) glDeleteBuffers(1, &this->vbo_weights_);
    if (this->ebo_) glDeleteBuffers(1, &this->ebo_);
    if (this->vbo_) glDeleteBuffers(1, &this->vbo_);
    if (this->vao_) glDeleteVertexArrays(1, &this->vao_);
    this->vao_ = this->vbo_ = this->ebo_ = this->vbo_weights_ = this->vbo_joint_weights_ =
        this->vbo_joint_indices_ = 0;
    this->index_count_ = 0;
}

void Mesh::upload(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices,
                  GLenum mode) {
    this->upload(vertices, indices, {}, {}, {}, mode);
}

void Mesh::upload(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices,
                  const std::vector<float>& weights, GLenum mode) {
    this->upload(vertices, indices, weights, {}, {}, mode);
}

void Mesh::upload(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices,
                  const std::vector<float>& weights, const std::vector<float>& joint_weights,
                  const std::vector<uint16_t>& joint_indices, GLenum mode) {
    this->draw_mode_ = mode;
    this->destroy();

    glGenVertexArrays(1, &this->vao_);
    glGenBuffers(1, &this->vbo_);
    glGenBuffers(1, &this->ebo_);

    glBindVertexArray(this->vao_);

    glBindBuffer(GL_ARRAY_BUFFER, this->vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                 vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(GLuint)),
                 indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, x));
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, nx));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, r));
    glEnableVertexAttribArray(2);

    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, u));
    glEnableVertexAttribArray(3);

    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tx));
    glEnableVertexAttribArray(4);

    if (!weights.empty()) {
        if (weights.size() != vertices.size() * 4) {
            auto log = get_logger();
            log->error(
                "Mesh::upload: splat weights size {} does not match vertices size {} * 4, "
                "skipping weight buffer",
                weights.size(), vertices.size());
        } else {
            glGenBuffers(1, &this->vbo_weights_);
            glBindBuffer(GL_ARRAY_BUFFER, this->vbo_weights_);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(weights.size() * sizeof(float)),
                         weights.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void*)0);
            glEnableVertexAttribArray(5);
        }
    }

    if (!joint_weights.empty() && !joint_indices.empty()) {
        if (joint_weights.size() != vertices.size() * 4 ||
            joint_indices.size() != vertices.size() * 4) {
            auto log = get_logger();
            log->error(
                "Mesh::upload: joint weights/indices size does not match vertices size * 4, "
                "skipping joint buffers");
        } else {
            glGenBuffers(1, &this->vbo_joint_weights_);
            glBindBuffer(GL_ARRAY_BUFFER, this->vbo_joint_weights_);
            glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(joint_weights.size() * sizeof(float)),
                         joint_weights.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void*)0);
            glEnableVertexAttribArray(6);

            glGenBuffers(1, &this->vbo_joint_indices_);
            glBindBuffer(GL_ARRAY_BUFFER, this->vbo_joint_indices_);
            glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(joint_indices.size() * sizeof(uint16_t)),
                         joint_indices.data(), GL_STATIC_DRAW);
            glVertexAttribIPointer(7, 4, GL_UNSIGNED_SHORT, 0, (void*)0);
            glEnableVertexAttribArray(7);
        }
    }

    glBindVertexArray(0);
    this->index_count_ = static_cast<GLsizei>(indices.size());
}

void Mesh::draw() const {
    if (!this->vao_) return;
    glBindVertexArray(this->vao_);
    glDrawElements(this->draw_mode_, this->index_count_, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

Mesh Mesh::create_cube(float size, float r, float g, float b) {
    float h = size * 0.5f;
    std::vector<Vertex> verts;
    std::vector<GLuint> idx;

    auto face = [&](float x, float y, float z, float nx, float ny, float nz, float t1x, float t1y,
                    float t1z, float t2x, float t2y, float t2z) {
        GLuint base = static_cast<GLuint>(verts.size());
        // corners: -t1-t2, +t1-t2, +t1+t2, -t1+t2
        verts.push_back({x + (-t1x - t2x) * h, y + (-t1y - t2y) * h, z + (-t1z - t2z) * h, nx, ny,
                         nz, r, g, b, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        verts.push_back({x + (t1x - t2x) * h, y + (t1y - t2y) * h, z + (t1z - t2z) * h, nx, ny, nz,
                         r, g, b, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        verts.push_back({x + (t1x + t2x) * h, y + (t1y + t2y) * h, z + (t1z + t2z) * h, nx, ny, nz,
                         r, g, b, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
        verts.push_back({x + (-t1x + t2x) * h, y + (-t1y + t2y) * h, z + (-t1z + t2z) * h, nx, ny,
                         nz, r, g, b, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});

        idx.push_back(base);
        idx.push_back(base + 1);
        idx.push_back(base + 2);
        idx.push_back(base);
        idx.push_back(base + 2);
        idx.push_back(base + 3);
    };

    // +X
    face(h, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0);
    // -X
    face(-h, 0, 0, -1, 0, 0, 0, 0, 1, 0, 1, 0);
    // +Y
    face(0, h, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1);
    // -Y
    face(0, -h, 0, 0, -1, 0, 1, 0, 0, 0, 0, 1);
    // +Z
    face(0, 0, h, 0, 0, 1, 1, 0, 0, 0, 1, 0);
    // -Z
    face(0, 0, -h, 0, 0, -1, 1, 0, 0, 0, 1, 0);

    Mesh mesh;
    mesh.upload(verts, idx);
    return mesh;
}

Mesh Mesh::create_pyramid(float base, float height, float r, float g, float b) {
    std::vector<Vertex> verts;
    std::vector<GLuint> idx;
    float hb = base * 0.5f;
    float hh = height * 0.5f;
    float ny = base / std::sqrt(base * base + 4.0f * height * height);
    float ns = 2.0f * height / std::sqrt(base * base + 4.0f * height * height);

    verts.push_back({0, hh, 0, 0, ny, ns, r, g, b, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
    verts.push_back(
        {-hb, -hh, -hb, 0, -1, 0, r * 0.7f, g * 0.7f, b * 0.7f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
    verts.push_back(
        {hb, -hh, -hb, 0, -1, 0, r * 0.7f, g * 0.7f, b * 0.7f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
    verts.push_back(
        {hb, -hh, hb, 0, -1, 0, r * 0.7f, g * 0.7f, b * 0.7f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});
    verts.push_back(
        {-hb, -hh, hb, 0, -1, 0, r * 0.7f, g * 0.7f, b * 0.7f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f});

    idx = {0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 1, 4, 1, 2, 3, 1, 3, 4};

    Mesh mesh;
    mesh.upload(verts, idx);
    return mesh;
}

Mesh Mesh::create_simple_tree(float r, float g, float b) {
    std::vector<Vertex> verts;
    std::vector<GLuint> idx;

    // Trunk: short brown cylinder rising from the ground.
    {
        const int seg = 10;
        const float trunk_r = 0.09f;
        const float trunk_top = 0.0f;
        const float trunk_bot = -0.5f;
        Vec3 col(0.42f, 0.3f, 0.16f);
        add_cylinder(verts, idx, seg, trunk_r, trunk_bot, trunk_top, trunk_r, col, col * 0.8f);
    }

    // Foliage: two stacked flattened spheres (low-poly lat/lon).
    const float fcy = 0.12f;
    const float radii[2] = {0.34f, 0.26f};
    const float centers[2] = {fcy, fcy + 0.26f};
    const int lat = 7, lon = 10;
    for (int s = 0; s < 2; ++s) {
        GLuint base = static_cast<GLuint>(verts.size());
        for (int i = 0; i < lat; ++i) {
            float th = static_cast<float>(robcraft::engine::math::kPi) * (i + 0.5f) / lat;
            for (int j = 0; j < lon; ++j) {
                float ph = static_cast<float>(robcraft::engine::math::kTwoPi) * j / lon;
                float x = radii[s] * std::sin(th) * std::cos(ph);
                float y = centers[s] + radii[s] * std::cos(th);
                float z = radii[s] * std::sin(th) * std::sin(ph);
                verts.push_back({x, y, z, x, (y - centers[s]) * 1.5f, z, r, g, b, 0, 0, 1, 0, 0});
            }
        }
        for (int i = 0; i < lat; ++i) {
            for (int j = 0; j < lon; ++j) {
                GLuint a = base + i * lon + j;
                GLuint b2 = base + i * lon + (j + 1) % lon;
                GLuint c2 = base + ((i + 1) % lat) * lon + j;
                GLuint d2 = base + ((i + 1) % lat) * lon + (j + 1) % lon;
                idx.push_back(a);
                idx.push_back(c2);
                idx.push_back(b2);
                idx.push_back(b2);
                idx.push_back(c2);
                idx.push_back(d2);
            }
        }
    }

    Mesh mesh;
    mesh.upload(verts, idx);
    return mesh;
}

Mesh Mesh::create_pine(float r, float g, float b) {
    std::vector<Vertex> verts;
    std::vector<GLuint> idx;

    // Trunk: thin brown cylinder.
    {
        const int seg = 8;
        const float trunk_r = 0.06f;
        Vec3 col(0.42f, 0.3f, 0.16f);
        add_cylinder(verts, idx, seg, trunk_r, -0.5f, 0.0f, trunk_r, col, col * 0.8f);
    }

    // Foliage: three stacked cones.
    const float cone_centers[3] = {0.08f, 0.26f, 0.44f};
    const float cone_radii[3] = {0.36f, 0.28f, 0.18f};
    const float cone_heights[3] = {0.34f, 0.3f, 0.26f};
    const int seg = 10;
    for (int s = 0; s < 3; ++s) {
        float cbot = cone_centers[s] - cone_heights[s] * 0.5f;
        float ctop = cone_centers[s] + cone_heights[s] * 0.5f;
        GLuint apex = static_cast<GLuint>(verts.size());
        verts.push_back({0, ctop, 0, 0, 1, 0, r, g, b, 0, 0, 1, 0, 0});
        for (int i = 0; i <= seg; ++i) {
            float a =
                static_cast<float>(i) / seg * static_cast<float>(robcraft::engine::math::kTwoPi);
            float c = std::cos(a) * cone_radii[s], sn = std::sin(a) * cone_radii[s];
            verts.push_back({c, cbot, sn, c, 0, sn, r * 0.9f, g * 0.9f, b * 0.9f, 0, 0, 1, 0, 0});
        }
        for (int i = 0; i < seg; ++i) {
            GLuint v0 = apex + 1 + i, v1 = apex + 1 + (i + 1) % seg;
            idx.push_back(apex);
            idx.push_back(v0);
            idx.push_back(v1);
        }
    }

    Mesh mesh;
    mesh.upload(verts, idx);
    return mesh;
}

Mesh Mesh::create_bush(float r, float g, float b) {
    std::vector<Vertex> verts;
    std::vector<GLuint> idx;
    const float centers[3][2] = {{0.0f, 0.0f}, {0.22f, 0.1f}, {-0.2f, 0.08f}};
    const float radii[3] = {0.3f, 0.22f, 0.2f};
    const int lat = 7, lon = 10;
    GLuint base = static_cast<GLuint>(verts.size());
    for (int s = 0; s < 3; ++s) {
        for (int i = 0; i < lat; ++i) {
            float th = static_cast<float>(robcraft::engine::math::kPi) * (i + 0.5f) / lat;
            for (int j = 0; j < lon; ++j) {
                float ph = static_cast<float>(robcraft::engine::math::kTwoPi) * j / lon;
                float x = centers[s][0] + radii[s] * std::sin(th) * std::cos(ph);
                float y = centers[s][1] + radii[s] * std::cos(th);
                float z = radii[s] * std::sin(th) * std::sin(ph);
                verts.push_back(
                    {x, y - 0.3f, z, x, (y - centers[s][1]) * 1.5f, z, r, g, b, 0, 0, 1, 0, 0});
            }
        }
        for (int i = 0; i < lat; ++i) {
            for (int j = 0; j < lon; ++j) {
                GLuint a = base + s * lat * lon + i * lon + j;
                GLuint b2 = base + s * lat * lon + i * lon + (j + 1) % lon;
                GLuint c2 = base + s * lat * lon + ((i + 1) % lat) * lon + j;
                GLuint d2 = base + s * lat * lon + ((i + 1) % lat) * lon + (j + 1) % lon;
                idx.push_back(a);
                idx.push_back(c2);
                idx.push_back(b2);
                idx.push_back(b2);
                idx.push_back(c2);
                idx.push_back(d2);
            }
        }
    }

    Mesh mesh;
    mesh.upload(verts, idx);
    return mesh;
}

}  // namespace robcraft::renderer
