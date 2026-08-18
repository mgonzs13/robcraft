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

#include <GL/glew.h>

#include <cstdint>
#include <vector>

namespace robcraft::renderer {

/** @brief A single vertex: position, normal, RGB color, UV, and tangent. */
struct Vertex {
    /** @brief Position (x, y, z). */
    float x, y, z;
    /** @brief Normal (nx, ny, nz). */
    float nx, ny, nz;
    /** @brief Color (r, g, b). */
    float r, g, b;
    /** @brief Texture coordinate (u, v). */
    float u, v;
    /** @brief Tangent (tx, ty, tz); bitangent = cross(normal, tangent). */
    float tx, ty, tz;
};

/** @brief OpenGL mesh: vertex/index buffers and draw support. */
class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& o) noexcept;
    Mesh& operator=(Mesh&& o) noexcept;

    /** @brief Upload vertex and index data to the GPU.
     *  @param vertices Interleaved vertex data.
     *  @param indices Element indices.
     *  @param mode Primitive mode (default GL_TRIANGLES). */
    void upload(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices,
                GLenum mode = GL_TRIANGLES);
    /** @brief Upload vertex/index data with optional per-vertex splat weights.
     *  @param vertices Interleaved vertex data.
     *  @param indices Element indices.
     *  @param weights Per-vertex blend weights, 4 floats per vertex (empty = none).
     *  @param mode Primitive mode. */
    void upload(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices,
                const std::vector<float>& weights, GLenum mode = GL_TRIANGLES);
    /** @brief Upload vertex/index data with optional splat weights and skinning data.
     *  @param vertices Interleaved vertex data.
     *  @param indices Element indices.
     *  @param weights Per-vertex splat blend weights, 4 floats per vertex (empty = none).
     *  @param joint_weights Per-vertex joint weights, 4 floats per vertex (empty = none).
     *  @param joint_indices Per-vertex joint indices, 4 u16 per vertex (empty = none).
     *  @param mode Primitive mode. */
    void upload(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices,
                const std::vector<float>& weights, const std::vector<float>& joint_weights,
                const std::vector<uint16_t>& joint_indices, GLenum mode = GL_TRIANGLES);
    /** @brief Bind and issue the indexed draw call. */
    void draw() const;
    /** @brief Delete all GPU buffers and reset the mesh. */
    void destroy();

    /** @return True if the VAO has been created. */
    bool valid() const { return this->vao_ != 0; }

    /** @brief Build a cube mesh.
     *  @param size Edge length.
     *  @param r Red component.
     *  @param g Green component.
     *  @param b Blue component.
     *  @return Uploaded cube mesh. */
    static Mesh create_cube(float size = 1.0f, float r = 0.5f, float g = 0.5f, float b = 0.5f);
    /** @brief Build a pyramid mesh.
     *  @param base Base edge length.
     *  @param height Pyramid height.
     *  @param r Red component.
     *  @param g Green component.
     *  @param b Blue component.
     *  @return Uploaded pyramid mesh. */
    static Mesh create_pyramid(float base, float height, float r, float g, float b);
    /** @brief Build a simple round-canopy tree (brown trunk + green foliage).
     *  Extent ~1.0 in unit space, centered near the ground at y = -0.5.
     *  @param r Foliage red component.
     *  @param g Foliage green component.
     *  @param b Foliage blue component.
     *  @return Uploaded tree mesh. */
    static Mesh create_simple_tree(float r = 0.18f, float g = 0.5f, float b = 0.14f);
    /** @brief Build a conical pine tree (brown trunk + stacked green cones).
     *  Extent ~1.0 in unit space, centered near the ground at y = -0.5.
     *  @param r Foliage red component.
     *  @param g Foliage green component.
     *  @param b Foliage blue component.
     *  @return Uploaded pine mesh. */
    static Mesh create_pine(float r = 0.14f, float g = 0.42f, float b = 0.16f);
    /** @brief Build a round bush (cluster of green spheres on the ground).
     *  Extent ~1.0 in unit space, bottom at y = -0.5.
     *  @param r Foliage red component.
     *  @param g Foliage green component.
     *  @param b Foliage blue component.
     *  @return Uploaded bush mesh. */
    static Mesh create_bush(float r = 0.22f, float g = 0.46f, float b = 0.14f);

private:
    /** @brief Vertex array object id. */
    GLuint vao_ = 0;
    /** @brief Vertex buffer object id. */
    GLuint vbo_ = 0;
    /** @brief Element buffer object id. */
    GLuint ebo_ = 0;
    /** @brief Number of indices to draw. */
    GLsizei index_count_ = 0;
    /** @brief Primitive mode used by the draw call. */
    GLenum draw_mode_ = GL_TRIANGLES;
    /** @brief Splat weight buffer object (terrain only). */
    GLuint vbo_weights_ = 0;
    /** @brief Joint weight buffer object (skinned models only). */
    GLuint vbo_joint_weights_ = 0;
    /** @brief Joint index buffer object (skinned models only). */
    GLuint vbo_joint_indices_ = 0;
};

}  // namespace robcraft::renderer
