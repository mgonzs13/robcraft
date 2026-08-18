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
 * @brief Renders a modal that collects a typed path.
 * @param title Popup title.
 * @param open In/out: whether the modal should be shown; set false on OK/Cancel.
 * @return The typed path when OK is pressed, otherwise nullopt.
 */
std::optional<std::string> imgui_path_modal(const char* title, bool& open);

}  // namespace robcraft::engine::io
