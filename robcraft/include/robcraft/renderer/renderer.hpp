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
#include <GLFW/glfw3.h>

#include <memory>
#include <string>

#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/camera.hpp"
#include "robcraft/renderer/mesh.hpp"
#include "robcraft/renderer/shader.hpp"
#include "robcraft/renderer/shader_state.hpp"
#include "robcraft/renderer/texture.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

/** @brief GLFW window plus the scene rendering pipeline. */
class Renderer {
public:
    Renderer() = default;
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    /** @brief Create the window and initialize OpenGL state.
     *  @param width Window width in pixels.
     *  @param height Window height in pixels.
     *  @param title Window title.
     *  @param hidden If true, create an invisible window (headless mode).
     *  @return True on success. */
    bool init(int width, int height, const std::string& title, bool hidden = false);
    /** @brief Tear down GLFW and shader resources. */
    void shutdown();

    /** @return True while the window should stay open. */
    bool is_running() const;
    /** @brief Poll and dispatch pending window events. */
    void poll_events();

    /** @brief Clear the color and depth buffers. */
    void begin_frame(); /** @brief Swap buffers and update frame timers. */
    void end_frame();

    /** @return Window width in pixels. */
    int width() const { return this->width_; }
    /** @return Window height in pixels. */
    int height() const { return this->height_; }

    /** @return Current GLFW time in seconds. */
    double time() const;
    /** @return Time since the last frame in seconds. */
    double delta_time() const;

    /** @brief Draw a mesh with a model matrix.
     *  @param model Model transform.
     *  @param mesh Mesh to draw. */
    void draw_entity(const Mat4& model, const Mesh& mesh);

    /** @return Reference to the scene camera. */
    Camera& camera() { return this->camera_; }
    /** @return Reference to the shared shader program. */
    Shader& shader() { return this->shader_; }

    /** @brief Sets the window icon from a PNG file.
     *  @param path Path to a PNG image file. */
    void set_window_icon(const std::string& path);

    /** @brief Sets all water shader defaults (normal map, reflection off).
     *  Used by the main render and robot-camera passes. */
    void set_water_defaults();

    /** @brief Binds the scene lighting (sun + ambient) and shadow toggle.
     *  @param lighting The world's scene lighting settings. */
    void set_lighting(const SceneLighting& lighting);
    /** @brief Uploads up to 8 point lights from the world's PointLight components.
     *  @param world The world to gather lights from. */
    void set_point_lights(const World& world);
    /** @brief Binds a sun-space depth texture as the shadow map.
     *  @param depth_tex Depth texture id (from FBO::depth_tex()).
     *  @param sun_view_proj Sun's projection * view matrix. */
    void set_shadow_map(GLuint depth_tex, const Mat4& sun_view_proj);

    /** @brief Binds the terrain albedo/normal texture arrays.
     *  @param albedo 5-layer albedo array (or invalid for flat-color fallback).
     *  @param normal 5-layer normal array (or invalid).
     *  @param use_splat True to enable splat sampling. */
    void set_terrain_textures(const Texture& albedo, const Texture& normal, bool use_splat);

    /** @brief Handle keyboard and mouse camera controls.
     *  @param dt Frame delta time in seconds. */
    void process_input(float dt);

    /** @brief GLFW scroll callback that accumulates wheel offsets for camera zoom.
     *  @param window The GLFW window that received the scroll event.
     *  @param xoffset Horizontal scroll offset.
     *  @param yoffset Vertical scroll offset. */
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

private:
    /** @brief GLFW window handle. */
    GLFWwindow* window_ = nullptr;
    /** @brief Window width in pixels. */
    int width_ = 800;
    /** @brief Window height in pixels. */
    int height_ = 600;
    /** @brief GLFW time of the previous frame. */
    double last_time_ = 0.0;
    /** @brief Time since the previous frame. */
    double delta_time_ = 0.0;
    /** @brief Accumulated scroll wheel offset, consumed each frame as camera zoom. */
    double scroll_offset_ = 0.0;

    /** @brief Refresh width/height from the GLFW framebuffer and refit the camera aspect. */
    void update_window_size();

    /** @brief Shared shader program. */
    Shader shader_;
    /** @brief Scene camera. */
    Camera camera_;
};

}  // namespace robcraft::renderer
