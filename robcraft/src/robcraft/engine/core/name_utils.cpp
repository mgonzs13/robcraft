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

#include "robcraft/engine/core/name_utils.hpp"

#include <cctype>
#include <string>

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

std::string sanitize_ros_name(const std::string& name) {
    std::string out;
    bool prev_underscore = false;
    for (char c : name) {
        char lower = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        bool valid = std::isalnum(static_cast<unsigned char>(lower)) || lower == '_';
        char out_char = valid ? lower : '_';
        if (out_char == '_') {
            if (prev_underscore) continue;
            prev_underscore = true;
        } else {
            prev_underscore = false;
        }
        out += out_char;
    }
    while (!out.empty() && out.front() == '_') out.erase(out.begin());
    while (!out.empty() && out.back() == '_') out.pop_back();
    if (!out.empty() && std::isdigit(static_cast<unsigned char>(out.front()))) {
        out.insert(out.begin(), '_');
    }
    return out;
}

std::string robot_base_name(const std::string& name, Entity entity) {
    std::string base = name;
    std::string suffix = "_" + std::to_string(entity);
    if (base.size() > suffix.size() &&
        base.compare(base.size() - suffix.size(), suffix.size(), suffix) == 0) {
        base = base.substr(0, base.size() - suffix.size());
    }
    std::string sanitized = sanitize_ros_name(base);
    return sanitized.empty() ? "robot" : sanitized;
}

std::string robot_namespace(const std::string& name, Entity entity, int index) {
    return robot_base_name(name, entity) + "_" + std::to_string(index);
}

}  // namespace robcraft::engine::core
