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

#include "robcraft/engine/io/file_dialog.hpp"

#include <array>
#include <cstdio>

namespace robcraft::engine::io {

using namespace robcraft::engine::io;

std::string run_command_output(const std::string& command) {
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return {};
    std::array<char, 256> buf;
    std::string out;
    while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) out += buf.data();
    // Exit status intentionally ignored: canceling a dialog yields empty stdout.
    pclose(pipe);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return out;
}

bool native_dialog_available() {
    return !run_command_output("command -v zenity").empty();
}

std::optional<std::string> native_open_file() {
    std::string out = run_command_output(
        "zenity --file-selection --file-filter=\"World files | *.world\" 2>/dev/null");
    if (out.empty()) return std::nullopt;
    return out;
}

std::optional<std::string> native_save_file() {
    std::string out = run_command_output(
        "zenity --file-selection --save --confirm-overwrite --file-filter=\"World files | "
        "*.world\" "
        "2>/dev/null");
    if (out.empty()) return std::nullopt;
    return out;
}

}  // namespace robcraft::engine::io
