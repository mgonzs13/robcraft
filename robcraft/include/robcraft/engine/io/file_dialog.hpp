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

#include <optional>
#include <string>

namespace robcraft::engine::io {

using namespace robcraft::engine::io;

/**
 * @brief Runs a command and returns its trimmed stdout, or empty on failure.
 * @param command The shell command to run.
 * @return stdout with trailing CR/LF characters removed, or "" if the command
 * fails or produces no output.
 */
std::string run_command_output(const std::string& command);

/**
 * @brief Whether a native file dialog can be shown on this system.
 * @return True if zenity is available.
 */
bool native_dialog_available();

/**
 * @brief Shows the native open-file dialog filtered to *.world.
 * @return The selected path, or nullopt on cancel/unavailable.
 */
std::optional<std::string> native_open_file();

/**
 * @brief Shows the native save-file dialog filtered to *.world.
 * @return The chosen path, or nullopt on cancel/unavailable.
 */
std::optional<std::string> native_save_file();

}  // namespace robcraft::engine::io
