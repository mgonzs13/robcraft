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

/** @brief OpenGL framebuffer object for offscreen rendering. */
class FBO {
public:
    FBO() = default;
    ~FBO();
    FBO(const FBO&) = delete;
    FBO& operator=(const FBO&) = delete;

    /** @brief Create the framebuffer with attached color/depth buffers.
     *  @param width Texture width in pixels.
     *  @param height Texture height in pixels.
     *  @return True if the framebuffer is complete. */
    bool create(int width, int height);
    /** @brief Delete the framebuffer and its attachments. */
    void destroy();
    /** @brief Bind the framebuffer for rendering. */
    void bind() const;
    /** @brief Bind the default (window) framebuffer. */
    void unbind() const;
    /** @brief Binds the color attachment to a texture unit for sampling.
     *  @param unit GL texture unit (GL_TEXTURE0..). */
    void bind_color(GLenum unit) const;
    /** @brief Create a depth-only framebuffer with a sampleable depth texture.
     *  @param width Texture width in pixels.
     *  @param height Texture height in pixels.
     *  @return True if the framebuffer is complete. */
    bool create_depth_only(int width, int height);
    /** @brief Read back the color buffer as RGB.
     *  @param out Vector resized to width * height * 3.
     *  @note glReadPixels yields rows bottom-up; this returns top-down rows. */
    void read_pixels_rgb(std::vector<uint8_t>& out) const;
    /** @brief Read back the depth buffer as linearization-ready floats.
     *  @param out Vector resized to width * height.
     *  @note glReadPixels yields rows bottom-up; this returns top-down rows. */
    void read_pixels_depth(std::vector<float>& out) const;

    /** @return True if the framebuffer has been created. */
    bool valid() const { return this->fbo_ != 0; }
    /** @return Framebuffer width in pixels. */
    int width() const { return this->width_; }
    /** @return Framebuffer height in pixels. */
    int height() const { return this->height_; }
    /** @return Color texture id. */
    GLuint color_tex() const { return this->color_tex_; }
    /** @return Sampleable depth texture id (0 if not created). */
    GLuint depth_tex() const { return this->depth_tex_; }

private:
    /** @brief Framebuffer object id. */
    GLuint fbo_ = 0;
    /** @brief Color attachment texture id. */
    GLuint color_tex_ = 0;
    /** @brief Depth renderbuffer id. */
    GLuint depth_rb_ = 0;
    /** @brief Sampleable depth texture id (used by create_depth_only). */
    GLuint depth_tex_ = 0;
    /** @brief Framebuffer width in pixels. */
    int width_ = 0;
    /** @brief Framebuffer height in pixels. */
    int height_ = 0;
};

/** @brief Flip a row-major RGB image buffer vertically in place.
 *  @param data Image pixels (width * height * 3 bytes, rows top-to-bottom).
 *  @param width Image width in pixels.
 *  @param height Image height in pixels. */
void flip_vertical_rgb(std::vector<uint8_t>& data, int width, int height);

}  // namespace robcraft::renderer
