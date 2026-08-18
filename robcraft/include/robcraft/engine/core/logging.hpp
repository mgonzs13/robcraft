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

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/**
 * @brief Initializes the named logger, creating it if needed.
 * @param name Logger name.
 * @return The initialized logger.
 */
std::shared_ptr<spdlog::logger> init_logger(const std::string& name = "robcraft");

/**
 * @brief Returns the named logger, creating it on first use.
 * @param name Logger name.
 * @return The requested logger.
 */
std::shared_ptr<spdlog::logger> get_logger(const std::string& name = "robcraft");

}  // namespace robcraft::engine::core
