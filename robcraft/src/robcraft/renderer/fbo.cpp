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

#include "robcraft/renderer/fbo.hpp"

#include <cstring>

namespace robcraft::renderer {

FBO::~FBO() {
    this->destroy();
}

bool FBO::create(int width, int height) {
    this->destroy();
    this->width_ = width;
    this->height_ = height;

    glGenFramebuffers(1, &this->fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, this->fbo_);

    glGenTextures(1, &this->color_tex_);
    glBindTexture(GL_TEXTURE_2D, this->color_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, this->color_tex_,
                           0);

    glGenRenderbuffers(1, &this->depth_rb_);
    glBindRenderbuffer(GL_RENDERBUFFER, this->depth_rb_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,
                              this->depth_rb_);

    bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    this->unbind();
    return ok;
}

void FBO::destroy() {
    if (this->depth_tex_) {
        glDeleteTextures(1, &this->depth_tex_);
        this->depth_tex_ = 0;
    }
    if (this->color_tex_) {
        glDeleteTextures(1, &this->color_tex_);
        this->color_tex_ = 0;
    }
    if (this->depth_rb_) {
        glDeleteRenderbuffers(1, &this->depth_rb_);
        this->depth_rb_ = 0;
    }
    if (this->fbo_) {
        glDeleteFramebuffers(1, &this->fbo_);
        this->fbo_ = 0;
    }
}

void FBO::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, this->fbo_);
}

void FBO::unbind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FBO::bind_color(GLenum unit) const {
    glActiveTexture(unit);
    glBindTexture(GL_TEXTURE_2D, this->color_tex_);
}

bool FBO::create_depth_only(int width, int height) {
    this->destroy();
    this->width_ = width;
    this->height_ = height;

    glGenFramebuffers(1, &this->fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, this->fbo_);

    glGenTextures(1, &this->depth_tex_);
    glBindTexture(GL_TEXTURE_2D, this->depth_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT,
                 GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, this->depth_tex_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    this->unbind();
    return ok;
}

void FBO::read_pixels_rgb(std::vector<uint8_t>& out) const {
    out.resize(this->width_ * this->height_ * 3);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, this->fbo_);
    glReadPixels(0, 0, this->width_, this->height_, GL_RGB, GL_UNSIGNED_BYTE, out.data());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    flip_vertical_rgb(out, this->width_, this->height_);
}

void FBO::read_pixels_depth(std::vector<float>& out) const {
    out.resize(static_cast<size_t>(this->width_) * this->height_);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, this->fbo_);
    glReadPixels(0, 0, this->width_, this->height_, GL_DEPTH_COMPONENT, GL_FLOAT, out.data());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    std::vector<float> row(static_cast<size_t>(this->width_));
    for (int y = 0; y < this->height_ / 2; ++y) {
        auto* top = out.data() + static_cast<size_t>(y) * this->width_;
        auto* bottom = out.data() + static_cast<size_t>(this->height_ - 1 - y) * this->width_;
        std::memcpy(row.data(), top, row.size() * sizeof(float));
        std::memcpy(top, bottom, row.size() * sizeof(float));
        std::memcpy(bottom, row.data(), row.size() * sizeof(float));
    }
}

void flip_vertical_rgb(std::vector<uint8_t>& data, int width, int height) {
    const size_t row_bytes = static_cast<size_t>(width) * 3;
    std::vector<uint8_t> row(row_bytes);
    for (int y = 0; y < height / 2; ++y) {
        auto* top = data.data() + static_cast<size_t>(y) * row_bytes;
        auto* bottom = data.data() + static_cast<size_t>(height - 1 - y) * row_bytes;
        std::memcpy(row.data(), top, row_bytes);
        std::memcpy(top, bottom, row_bytes);
        std::memcpy(bottom, row.data(), row_bytes);
    }
}

}  // namespace robcraft::renderer
