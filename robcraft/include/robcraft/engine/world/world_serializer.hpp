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

#include "robcraft/engine/world/world.hpp"

namespace robcraft::engine::world {

using namespace robcraft::engine::world;

/** @brief Static helpers to load and save worlds in the custom text format. */
class WorldSerializer {
public:
    /**
     * @brief Saves a world to a .world file, plus terrain if present.
     * @param world The world to serialize.
     * @param path Output file path.
     * @return True on success.
     */
    static bool save(const World& world, const std::string& path);
    /**
     * @brief Loads a world from a .world file.
     * @param world The world to populate.
     * @param path Input file path.
     * @return True on success.
     */
    static bool load(World& world, const std::string& path);
};

}  // namespace robcraft::engine::world
