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

#include "robcraft/renderer/camera_controls.hpp"

#include <cmath>

#include "robcraft/renderer/camera.hpp"

namespace robcraft::renderer {

void handle_camera_input(GLFWwindow* window, Camera& cam, double& scroll_offset, float move_speed,
                         float orbit_scale, bool keyboard_enabled, bool orbit_enabled,
                         bool zoom_enabled) {
    if (keyboard_enabled) {
        // Don't move the camera while a shortcut modifier is held (Ctrl+S etc.).
        bool mod_down = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
        if (!mod_down) {
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) cam.pan(move_speed, 0.0f);
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) cam.pan(-move_speed, 0.0f);
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) cam.pan(0.0f, -move_speed);
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) cam.pan(0.0f, move_speed);
            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) cam.move_world_up(-move_speed);
            if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) cam.move_world_up(move_speed);
        }
    }

    {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        static double prev_mx = mx, prev_my = my;
        float dx = static_cast<float>(mx - prev_mx);
        float dy = static_cast<float>(my - prev_my);
        prev_mx = mx;
        prev_my = my;
        if (orbit_enabled && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
            float dist = static_cast<float>((cam.position() - cam.orbit_target()).length());
            cam.orbit(dx * orbit_scale, -dy * orbit_scale, dist);
        }
    }

    if (zoom_enabled && scroll_offset != 0.0) {
        float factor = std::pow(1.15f, static_cast<float>(-scroll_offset));
        if (cam.has_orbit_target()) {
            cam.zoom_by_factor(factor);
        } else {
            const float zoom_step = 15.0f;
            cam.move_forward(static_cast<float>(scroll_offset) * zoom_step);
        }
        scroll_offset = 0.0;
    }
}

}  // namespace robcraft::renderer
