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

#include <string>

namespace robcraft::renderer {

/** @brief Loads a PNG from disk and sets it as the GLFW window icon.
 *  Pixels are decoded top-down (vertical flip disabled), matching the
 *  orientation window managers expect for taskbar/window icons.
 *  @param window GLFW window whose icon to set (ignored if null).
 *  @param path Path to a PNG image file.
 */
void set_window_icon(GLFWwindow* window, const std::string& path);

}  // namespace robcraft::renderer
