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

#include "robcraft/engine/core/entity.hpp"

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/**
 * @brief Converts a raw name into a valid ROS 2 namespace part.
 * @param name The raw name to sanitize.
 * @return Lowercased, underscore-safe identifier; empty if nothing valid remains.
 */
std::string sanitize_ros_name(const std::string& name);

/**
 * @brief Derives the base namespace for a robot from its name and entity id.
 * @param name The robot's Name component value (empty if absent).
 * @param entity The robot's entity id.
 * @return The base name with any trailing "_<entity_id>" suffix stripped, or "robot".
 */
std::string robot_base_name(const std::string& name, Entity entity);

/**
 * @brief Builds the full robot namespace from a name, entity id, and index.
 * @param name The robot's Name component value (empty if absent).
 * @param entity The robot's entity id.
 * @param index 1-based index among robots sharing the same base name.
 * @return The namespace, e.g. "robot_mike_1".
 */
std::string robot_namespace(const std::string& name, Entity entity, int index);

}  // namespace robcraft::engine::core
