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

#include "robcraft/renderer/renderer.hpp"

#include <iostream>

#include "robcraft/engine/core/logging.hpp"
#include "robcraft/renderer/camera_controls.hpp"
#include "robcraft/renderer/window_icon.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::core;

static void glfw_error_callback(int code, const char* desc) {
    std::cerr << "GLFW error " << code << ": " << desc << std::endl;
}

void Renderer::scroll_callback(GLFWwindow* window, double /*xoffset*/, double yoffset) {
    auto* self = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (self) self->scroll_offset_ += yoffset;
}

Renderer::~Renderer() {
    this->shutdown();
}

bool Renderer::init(int width, int height, const std::string& title, bool hidden) {
    this->width_ = width;
    this->height_ = height;

    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit()) {
        auto log = get_logger();
        log->error("Failed to initialize GLFW");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    if (hidden) {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    }

    this->window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!this->window_) {
        auto log = get_logger();
        log->error("Failed to create GLFW window");
        glfwTerminate();
        return false;
    }
    if (!hidden) {
        glfwMaximizeWindow(this->window_);
    }

    glfwMakeContextCurrent(this->window_);
    glfwSetWindowUserPointer(this->window_, this);
    glfwSetScrollCallback(this->window_, Renderer::scroll_callback);
    glfwSwapInterval(1);

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        auto log = get_logger();
        log->error("Failed to initialize GLEW: {}",
                   reinterpret_cast<const char*>(glewGetErrorString(err)));
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_CLIP_DISTANCE0);
    glCullFace(GL_BACK);
    glClearColor(0.7f, 0.8f, 0.9f, 1.0f);

    if (!this->shader_.compile(Shader::default_vertex(), Shader::default_fragment())) {
        auto log = get_logger();
        log->error("Failed to compile default shader");
        return false;
    }

    this->camera_.set_perspective(60.0f, static_cast<float>(width) / height, 0.1f, 500.0f);

    this->last_time_ = glfwGetTime();

    auto log = get_logger();
    log->info("Renderer initialized: {}x{} OpenGL {}", width, height,
              reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    return true;
}

void Renderer::set_window_icon(const std::string& path) {
    robcraft::renderer::set_window_icon(this->window_, path);
}

void Renderer::shutdown() {
    this->shader_.destroy();
    if (this->window_) {
        glfwDestroyWindow(this->window_);
        this->window_ = nullptr;
        glfwTerminate();
    }
}

bool Renderer::is_running() const {
    return this->window_ && !glfwWindowShouldClose(this->window_);
}

void Renderer::poll_events() {
    glfwPollEvents();
}

void Renderer::begin_frame() {
    this->update_window_size();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    this->shader_.use();
    this->shader_.set_uniform("uTime", static_cast<float>(this->time()));
    auto cp = this->camera_.position();
    this->shader_.set_uniform("uCameraPos", static_cast<float>(cp.x), static_cast<float>(cp.y),
                              static_cast<float>(cp.z));

    this->set_water_defaults();
}

void Renderer::set_water_defaults() {
    this->shader_.use();
    robcraft::renderer::set_water_defaults(this->shader_);
}

void Renderer::update_window_size() {
    if (!this->window_) return;
    int w, h;
    glfwGetFramebufferSize(this->window_, &w, &h);
    if (w < 1) w = 1;
    if (h < 1) h = 1;
    if (w == this->width_ && h == this->height_) return;
    this->width_ = w;
    this->height_ = h;
    this->camera_.set_perspective(this->camera_.fov(), static_cast<float>(w) / h,
                                  this->camera_.near_plane(), this->camera_.far_plane());
}

void Renderer::end_frame() {
    glfwSwapBuffers(this->window_);

    double now = glfwGetTime();
    this->delta_time_ = now - this->last_time_;
    this->last_time_ = now;
}

double Renderer::time() const {
    return glfwGetTime();
}

double Renderer::delta_time() const {
    return this->delta_time_;
}

void Renderer::draw_entity(const Mat4& model, const Mesh& mesh) {
    this->shader_.set_uniform("uModel", model.ptr());
    mesh.draw();
}

void Renderer::set_lighting(const SceneLighting& lighting) {
    this->shader_.use();
    robcraft::renderer::upload_scene_lighting(this->shader_, lighting);
}

void Renderer::set_point_lights(const World& world) {
    this->shader_.use();
    robcraft::renderer::upload_point_lights(this->shader_, world);
}

void Renderer::set_shadow_map(GLuint depth_tex, const Mat4& sun_view_proj) {
    this->shader_.use();
    glActiveTexture(GL_TEXTURE6);
    glBindTexture(GL_TEXTURE_2D, depth_tex);
    this->shader_.set_uniform("uShadowMap", 6);
    this->shader_.set_uniform("uSunViewProj", sun_view_proj.ptr());
}

void Renderer::set_terrain_textures(const Texture& albedo, const Texture& normal, bool use_splat) {
    this->shader_.use();
    robcraft::renderer::bind_terrain_textures(this->shader_, albedo, normal, use_splat);
}

void Renderer::process_input(float dt) {
    // move_speed = 30 m/s; orbit scale = rot_speed * 0.3 = 4 * dt * 0.3.
    robcraft::renderer::handle_camera_input(this->window_, this->camera_, this->scroll_offset_,
                                            30.0f * dt, 4.0f * 0.3f * dt, true, true, true);
}

}  // namespace robcraft::renderer
