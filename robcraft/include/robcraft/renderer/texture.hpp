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
#include <vector>

namespace robcraft::renderer {

/** @brief Decoded RGBA image held in CPU memory. */
struct Image {
    /** @brief Pixel width. */
    int width = 0;
    /** @brief Pixel height. */
    int height = 0;
    /** @brief Color channels (3 = RGB, 4 = RGBA). */
    int channels = 0;
    /** @brief Raw pixel data, width*height*channels bytes. */
    std::vector<unsigned char> pixels;
};

/** @brief Returns true if any pixel has an alpha byte < 255.
 *  @param img Decoded RGBA image.
 *  @return True when the image contains any transparency. */
bool image_has_alpha(const Image& img);

/**
 * @brief Decodes an image file from disk via stb_image.
 * @param path File path.
 * @param out Decoded image.
 * @return True on success.
 */
bool load_image_file(const std::string& path, Image& out);

/** @brief OpenGL texture object (2D or 2D array), move-only RAII. */
class Texture {
public:
    Texture() = default;
    ~Texture();
    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& o) noexcept;
    Texture& operator=(Texture&& o) noexcept;

    /** @brief Creates a GL_TEXTURE_2D from an image file.
     *  @param path Image file path.
     *  @return Uploaded texture, or invalid on failure. */
    static Texture create_2d(const std::string& path);
    /** @brief Creates a GL_TEXTURE_2D_ARRAY from equal-sized image files.
     *  @param paths Layer image paths (all must match in size).
     *  @return Uploaded array texture, or invalid on mismatch/failure. */
    static Texture create_2d_array(const std::vector<std::string>& paths);
    /** @brief Creates a 1x1 flat-normal texture (RGBA 128,128,255,255).
     *  @return Uploaded neutral normal texture. */
    static Texture create_neutral_normal();
    /** @brief Builds a deterministic 256x256 neutral surface-detail noise texture.
     *  Mean ~1.0 so `texture * vertexColor` keeps the material tint from the
     *  vertex color (tinting the noise itself would double-darken materials).
     *  @param seed_name Material/model name used to seed the variation.
     *  @return Uploaded RGB texture. */
    static Texture create_procedural(const std::string& seed_name);
    /** @brief Generates a deterministic 256x256 tiling ripple normal map.
     *  RGB encodes a world-space normal (r = +X, g = +Y, b = +Z) in [0,1].
     *  Seamless: every wave completes an integer number of cycles per tile.
     *  @return CPU-side image ready for upload. */
    static Image generate_water_normal_image();
    /** @brief Uploads generate_water_normal_image() as a GL_TEXTURE_2D.
     *  @return Uploaded texture, or invalid on failure. */
    static Texture create_water_normal();

    /** @brief Binds the texture to a texture unit.
     *  @param unit GL texture unit index (GL_TEXTURE0..). */
    void bind(GLenum unit) const;
    /** @brief Deletes the GL texture and resets the object. */
    void destroy();
    /** @return True if a GL texture id exists. */
    bool valid() const { return this->id_ != 0; }
    /** @return GL texture id. */
    GLuint id() const { return this->id_; }

private:
    /** @brief Uploads an array of decoded images as a 2D or 2D-array texture. */
    void upload(const std::vector<Image>& images, bool as_array);

    /** @brief GL texture id. */
    GLuint id_ = 0;
    /** @brief Pixel width. */
    int width_ = 0;
    /** @brief Pixel height. */
    int height_ = 0;
    /** @brief Layer count (1 for 2D). */
    int layers_ = 0;
    /** @brief True if this is a 2D array texture (affects bind target). */
    bool is_array_ = false;
    /** @brief True if any uploaded pixel had alpha < 255 (foliage cutout). */
    bool has_alpha_ = false;
};

}  // namespace robcraft::renderer
