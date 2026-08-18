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

namespace robcraft::engine::io {

using namespace robcraft::engine::io;

/**
 * @brief Returns the directory prefix of a path.
 * @param path File path.
 * @return Directory prefix including the trailing '/', or empty if none.
 */
inline std::string directory_of(const std::string& path) {
    auto pos = path.find_last_of('/');
    return pos == std::string::npos ? std::string() : path.substr(0, pos + 1);
}

/**
 * @brief Resolves a sibling .mtl path for an OBJ file.
 * @param obj_path OBJ file path.
 * @return Preferred .mtl path (same directory, same basename).
 */
inline std::string sibling_mtl_path(const std::string& obj_path) {
    auto slash = obj_path.find_last_of('/');
    auto dot = obj_path.find_last_of('.');
    std::string base = (dot != std::string::npos && (slash == std::string::npos || dot > slash))
                           ? obj_path.substr(0, dot)
                           : obj_path;
    return base + ".mtl";
}

}  // namespace robcraft::engine::io
