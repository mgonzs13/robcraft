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

#include "robcraft/renderer/texture.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/math/constants.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;

namespace {
/** @brief FNV-1a 32-bit hash of a string.
 *  @param s Input string.
 *  @return Deterministic 32-bit hash value. */
uint32_t fnv1a(const std::string& s) {
    uint32_t h = 2166136261u;
    for (unsigned char c : s) {
        h ^= c;
        h *= 16777619u;
    }
    return h;
}
}  // namespace

/**
 * @brief Scans pixel data for any alpha byte below 255.
 * @param img Decoded RGBA image (channels must be 4).
 * @return True if the image contains any transparency.
 */
bool image_has_alpha(const Image& img) {
    for (size_t i = 0; i + 3 < img.pixels.size(); i += 4) {
        if (img.pixels[i + 3] < 255) return true;
    }
    return false;
}

bool load_image_file(const std::string& path, Image& out) {
    stbi_set_flip_vertically_on_load(true);
    int w = 0, h = 0, ch = 0;
    unsigned char* px = stbi_load(path.c_str(), &w, &h, &ch, 4);
    if (!px) return false;
    out.width = w;
    out.height = h;
    out.channels = 4;
    out.pixels.assign(px, px + static_cast<size_t>(w) * h * 4);
    stbi_image_free(px);
    return true;
}

Texture::~Texture() {
    this->destroy();
}

Texture::Texture(Texture&& o) noexcept
    : id_(o.id_),
      width_(o.width_),
      height_(o.height_),
      layers_(o.layers_),
      is_array_(o.is_array_),
      has_alpha_(o.has_alpha_) {
    o.id_ = 0;
    o.width_ = o.height_ = o.layers_ = 0;
    o.is_array_ = false;
    o.has_alpha_ = false;
}

Texture& Texture::operator=(Texture&& o) noexcept {
    if (this != &o) {
        this->destroy();
        this->id_ = o.id_;
        this->width_ = o.width_;
        this->height_ = o.height_;
        this->layers_ = o.layers_;
        this->is_array_ = o.is_array_;
        this->has_alpha_ = o.has_alpha_;
        o.id_ = 0;
        o.width_ = o.height_ = o.layers_ = 0;
        o.is_array_ = false;
        o.has_alpha_ = false;
    }
    return *this;
}

void Texture::destroy() {
    if (this->id_) glDeleteTextures(1, &this->id_);
    this->id_ = 0;
    this->width_ = this->height_ = this->layers_ = 0;
    this->is_array_ = false;
    this->has_alpha_ = false;
}

void Texture::bind(GLenum unit) const {
    glActiveTexture(unit);
    glBindTexture(this->is_array_ ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D, this->id_);
}

void Texture::upload(const std::vector<Image>& images, bool as_array) {
    this->destroy();
    if (images.empty()) return;

    const int w = images[0].width;
    const int h = images[0].height;
    for (const auto& img : images) {
        if (img.width != w || img.height != h || img.channels != 4) {
            auto log = get_logger();
            log->error("Texture array layer size mismatch: {}x{} vs {}x{}", img.width, img.height,
                       w, h);
            return;
        }
    }
    // Array textures (terrain splat) are opaque today; foliage is single-image
    // 2D, so scanning layer 0 is sufficient. Revisit if mixed-alpha arrays arrive.
    this->has_alpha_ = image_has_alpha(images[0]);

    GLenum target = as_array ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D;
    glGenTextures(1, &this->id_);
    glBindTexture(target, this->id_);
    if (as_array) {
        glTexImage3D(target, 0, GL_RGBA8, w, h, static_cast<GLsizei>(images.size()), 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, nullptr);
        for (size_t i = 0; i < images.size(); ++i) {
            glTexSubImage3D(target, 0, 0, 0, static_cast<GLint>(i), w, h, 1, GL_RGBA,
                            GL_UNSIGNED_BYTE, images[i].pixels.data());
        }
    } else {
        glTexImage2D(target, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     images[0].pixels.data());
    }
    glGenerateMipmap(target);
    glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(target, 0);

    this->width_ = w;
    this->height_ = h;
    this->layers_ = as_array ? static_cast<int>(images.size()) : 1;
    this->is_array_ = as_array;
}

Texture Texture::create_2d(const std::string& path) {
    Image img;
    if (!load_image_file(path, img)) {
        auto log = get_logger();
        log->error("Failed to load texture: {}", path);
        return Texture();
    }
    Texture tex;
    tex.upload({img}, false);
    return tex;
}

Texture Texture::create_neutral_normal() {
    Image img;
    img.width = 1;
    img.height = 1;
    img.channels = 4;
    img.pixels = {128, 128, 255, 255};
    Texture tex;
    tex.upload({img}, false);
    return tex;
}

Texture Texture::create_2d_array(const std::vector<std::string>& paths) {
    std::vector<Image> images;
    for (const auto& p : paths) {
        Image img;
        if (!load_image_file(p, img)) {
            auto log = get_logger();
            log->error("Failed to load texture layer: {}", p);
            return Texture();
        }
        images.push_back(std::move(img));
    }
    Texture tex;
    tex.upload(images, true);
    return tex;
}

Texture Texture::create_procedural(const std::string& seed_name) {
    // Neutral surface-detail noise (mean ~1.0): the shader multiplies the albedo
    // texture by the vertex color, which already carries the material tint, so
    // tinting the noise too would darken materials twice over (e.g. 0.12*0.12).
    const int kSize = 256;
    Image img;
    img.width = kSize;
    img.height = kSize;
    img.channels = 4;
    img.pixels.resize(static_cast<size_t>(kSize) * kSize * 4);
    for (int y = 0; y < kSize; ++y) {
        for (int x = 0; x < kSize; ++x) {
            std::string key = seed_name + ":" + std::to_string(x) + ":" + std::to_string(y);
            uint32_t h = fnv1a(key);
            // Per-channel noise comes from different bytes of the same pixel hash.
            float fr = (h & 0xFF) / 255.0f;
            float fg = ((h >> 8) & 0xFF) / 255.0f;
            float fb = ((h >> 16) & 0xFF) / 255.0f;
            float grad = static_cast<float>(0.92 + 0.08 * std::sin((x + y) / 48.0));
            float cr = std::min(std::max(0.9f + 0.2f * fr, 0.0f), 1.0f) * grad;
            float cg = std::min(std::max(0.9f + 0.2f * fg, 0.0f), 1.0f) * grad;
            float cb = std::min(std::max(0.9f + 0.2f * fb, 0.0f), 1.0f) * grad;
            size_t p = static_cast<size_t>(y * kSize + x) * 4;
            img.pixels[p + 0] = static_cast<unsigned char>(cr * 255.0f);
            img.pixels[p + 1] = static_cast<unsigned char>(cg * 255.0f);
            img.pixels[p + 2] = static_cast<unsigned char>(cb * 255.0f);
            img.pixels[p + 3] = 255;
        }
    }
    Texture tex;
    tex.upload({img}, false);
    return tex;
}

Image Texture::generate_water_normal_image() {
    const int kSize = 256;
    Image img;
    img.width = kSize;
    img.height = kSize;
    img.channels = 4;
    img.pixels.resize(static_cast<size_t>(kSize) * kSize * 4);

    // Directional waves: integer cycles per tile along x and z (integer ->
    // seamless), amplitude. Deterministic by construction.
    struct Wave {
        int cx, cz;
        float amp;
    };
    const Wave waves[] = {
        {1, 0, 0.35f}, {1, 1, 0.30f}, {2, 1, 0.22f}, {2, 2, 0.18f}, {3, 1, 0.12f}, {4, 0, 0.08f},
    };
    const float kSpan = 8.0f;  // world meters spanned by one tile
    const float kTwoPi = static_cast<float>(robcraft::engine::math::kTwoPi);

    // Sample the full tile including the far edge (x = kSize-1 lands on the
    // period boundary) so texel 255 equals the wrapped copy of texel 0.
    for (int y = 0; y < kSize; ++y) {
        for (int x = 0; x < kSize; ++x) {
            float px = static_cast<float>(x) / (kSize - 1) * kSpan;
            float pz = static_cast<float>(y) / (kSize - 1) * kSpan;
            float h = 0.0f, dhdx = 0.0f, dhdz = 0.0f;
            for (const auto& w : waves) {
                float kx = kTwoPi * static_cast<float>(w.cx) / kSpan;
                float kz = kTwoPi * static_cast<float>(w.cz) / kSpan;
                float phase = kx * px + kz * pz;
                float sinp = std::sin(phase);
                h += w.amp * std::cos(phase);
                dhdx -= w.amp * kx * sinp;
                dhdz -= w.amp * kz * sinp;
            }
            // World-space normal from the height gradient.
            float nx = -dhdx;
            float nz = -dhdz;
            float ny = 1.0f;
            float inv = 1.0f / std::sqrt(nx * nx + ny * ny + nz * nz);
            size_t p = static_cast<size_t>(y * kSize + x) * 4;
            img.pixels[p + 0] = static_cast<unsigned char>((nx * inv * 0.5f + 0.5f) * 255.0f);
            img.pixels[p + 1] = static_cast<unsigned char>((ny * inv * 0.5f + 0.5f) * 255.0f);
            img.pixels[p + 2] = static_cast<unsigned char>((nz * inv * 0.5f + 0.5f) * 255.0f);
            img.pixels[p + 3] = 255;
        }
    }
    return img;
}

Texture Texture::create_water_normal() {
    Texture tex;
    tex.upload({Texture::generate_water_normal_image()}, false);
    return tex;
}

}  // namespace robcraft::renderer
