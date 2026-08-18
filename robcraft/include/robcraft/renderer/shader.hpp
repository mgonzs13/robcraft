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

#include <string>

namespace robcraft::renderer {

/** @brief Wraps an OpenGL shader program (vertex + fragment stages). */
class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& o) noexcept;
    Shader& operator=(Shader&& o) noexcept;

    /** @brief Compile and link a shader program from source strings.
     *  @param vertex_src Vertex shader source.
     *  @param fragment_src Fragment shader source.
     *  @return True on successful compile and link. */
    bool compile(const std::string& vertex_src, const std::string& fragment_src);

    /** @brief Bind this program for subsequent draw calls. */
    void use() const;

    /** @brief Delete the underlying program and reset the id. */
    void destroy();

    /** @return OpenGL program id (0 when not compiled). */
    GLuint id() const { return this->program_; }

    /** @return True if a program is currently linked. */
    bool valid() const { return this->program_ != 0; }

    /** @brief Set a float uniform.
     *  @param name Uniform name in the shader.
     *  @param value Float value. */
    void set_uniform(const std::string& name, float value) const;

    /** @brief Set an int uniform.
     *  @param name Uniform name in the shader.
     *  @param value Int value. */
    void set_uniform(const std::string& name, int value) const;

    /** @brief Set a mat4 uniform.
     *  @param name Uniform name in the shader.
     *  @param mat4 Column-major 4x4 matrix pointer. */
    void set_uniform(const std::string& name, const float* mat4) const;

    /** @brief Set a vec3 uniform.
     *  @param name Uniform name in the shader.
     *  @param x First component.
     *  @param y Second component.
     *  @param z Third component. */
    void set_uniform(const std::string& name, float x, float y, float z) const;

    /** @brief Sets a vec4 uniform.
     *  @param name Uniform name.
     *  @param x First component.
     *  @param y Second component.
     *  @param z Third component.
     *  @param w Fourth component. */
    void set_uniform(const std::string& name, float x, float y, float z, float w) const;

    /** @return Built-in vertex shader source string. */
    static const char* default_vertex();

    /** @return Built-in fragment shader source string. */
    static const char* default_fragment();

private:
    /** @brief Compile a single shader stage.
     *  @param type GL shader stage type.
     *  @param src Shader source string.
     *  @return Compiled shader id, or 0 on failure. */
    GLuint compile_stage(GLenum type, const std::string& src);

    /** @brief Look up a uniform location.
     *  @param name Uniform name in the shader.
     *  @return Uniform location, or -1 when not found. */
    GLint uniform_location(const std::string& name) const;

    /** @brief OpenGL program id, 0 when invalid. */
    GLuint program_ = 0;
};

}  // namespace robcraft::renderer
