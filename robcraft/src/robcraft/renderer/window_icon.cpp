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

#include "robcraft/renderer/window_icon.hpp"

#include <stb_image.h>

#include "robcraft/engine/core/logging.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::core;

void set_window_icon(GLFWwindow* window, const std::string& path) {
    if (!window) return;

    // GLFW icons are top-down; textures elsewhere load flipped for OpenGL, so
    // disable the global flip for this decode and restore it afterward.
    stbi_set_flip_vertically_on_load(false);
    int w = 0, h = 0, ch = 0;
    unsigned char* px = stbi_load(path.c_str(), &w, &h, &ch, 4);
    stbi_set_flip_vertically_on_load(true);
    if (!px) {
        auto log = get_logger();
        log->warn("Failed to load window icon: {}", path);
        return;
    }

    GLFWimage image{w, h, px};
    glfwSetWindowIcon(window, 1, &image);
    stbi_image_free(px);
}

}  // namespace robcraft::renderer
