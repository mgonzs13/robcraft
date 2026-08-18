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

#include "robcraft/engine/core/logging.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

std::shared_ptr<spdlog::logger> init_logger(const std::string& name) {
    auto logger = spdlog::stdout_color_mt(name);
    logger->set_level(spdlog::level::info);
    logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    return logger;
}

std::shared_ptr<spdlog::logger> get_logger(const std::string& name) {
    auto logger = spdlog::get(name);
    if (!logger) {
        logger = init_logger(name);
    }
    return logger;
}

}  // namespace robcraft::engine::core
