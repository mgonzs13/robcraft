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

#include "robcraft/engine/core/data_path.hpp"

#include <filesystem>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

std::string data_share_dir() {
    try {
        return ament_index_cpp::get_package_share_directory("robcraft");
    } catch (const std::exception&) {
        return {};
    }
}

std::string resolve_data_path(const std::string& rel) {
    return resolve_data_path(rel, data_share_dir());
}

std::string resolve_data_path(const std::string& rel, const std::string& share_root) {
    std::filesystem::path p(rel);
    if (p.is_absolute() || std::filesystem::exists(p)) return rel;
    if (!share_root.empty()) {
        std::filesystem::path candidate = std::filesystem::path(share_root) / p;
        if (std::filesystem::exists(candidate)) return candidate.string();
    }
    return rel;
}

}  // namespace robcraft::engine::core
