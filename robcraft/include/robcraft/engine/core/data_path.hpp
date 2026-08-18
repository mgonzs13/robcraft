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

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/**
 * @brief Returns the installed package share directory, or an empty string when unavailable.
 * @return Absolute path to share/robcraft, or "" if the package is not installed/indexed.
 */
std::string data_share_dir();

/**
 * @brief Resolves a possibly-relative data path, preferring CWD then the package share dir.
 * @param rel Relative (or absolute) path to resolve.
 * @return Absolute path in the package share dir when rel is relative and not found in CWD,
 *         otherwise rel unchanged.
 */
std::string resolve_data_path(const std::string& rel);

/**
 * @brief Resolves a path against CWD and an explicit share root (used by tests).
 * @param rel Relative (or absolute) path to resolve.
 * @param share_root Directory to fall back to when rel is not found in CWD.
 * @return share_root/rel when found there, otherwise rel unchanged.
 */
std::string resolve_data_path(const std::string& rel, const std::string& share_root);

}  // namespace robcraft::engine::core
