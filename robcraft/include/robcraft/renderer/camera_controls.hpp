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

#include <GLFW/glfw3.h>

namespace robcraft::renderer {

class Camera;

/** @brief Applies keyboard/mouse camera controls for one frame.
 *  @param window The GLFW window.
 *  @param cam The camera to move.
 *  @param scroll_offset Accumulated scroll offset; consumed and reset when
 *         zoom_enabled and non-zero.
 *  @param move_speed Pan/up speed in units per second (dt-scaled by caller).
 *  @param orbit_scale Orbit sensitivity in radians per pixel (dt-scaled by caller).
 *  @param keyboard_enabled Whether WASD/QE panning is active this frame.
 *  @param orbit_enabled Whether right-drag orbiting is active this frame.
 *  @param zoom_enabled Whether scroll zooming is active this frame. */
void handle_camera_input(GLFWwindow* window, Camera& cam, double& scroll_offset, float move_speed,
                         float orbit_scale, bool keyboard_enabled, bool orbit_enabled,
                         bool zoom_enabled);

}  // namespace robcraft::renderer
