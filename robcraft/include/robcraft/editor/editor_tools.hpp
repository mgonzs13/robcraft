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

#include <string>

namespace robcraft::editor {

class EditorApp;

/** @brief Creates, frames, and loads/saves editor worlds. */
class EditorTools {
public:
    /** @brief Constructs the tools.
     *  @param app The owning editor application. */
    explicit EditorTools(EditorApp& app);

    /** @brief Creates a fresh world with the given terrain dimensions and frames the camera. */
    void create_world(int width, int height, double cell_size);
    /** @brief Frames the editor camera to look at the terrain center. */
    void frame_world();
    /** @brief Marks the world as having unsaved changes. */
    void mark_modified();
    /** @brief Loads a world from a .world file.
     *  @param path The .world file path.
     *  @return True on success. */
    bool open_world(const std::string& path);
    /** @brief Saves the world to a .world file.
     *  @param path The destination file path.
     *  @return True on success. */
    bool save_world(const std::string& path);

private:
    /** @brief The owning editor application. */
    EditorApp& app_;
};

}  // namespace robcraft::editor
