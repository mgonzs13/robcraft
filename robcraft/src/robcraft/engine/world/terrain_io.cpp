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

#include "robcraft/engine/world/terrain_io.hpp"

#include <cstdint>
#include <fstream>

#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/world/terrain.hpp"

namespace robcraft::engine::world {

using namespace robcraft::engine::world;
using namespace robcraft::engine::core;

bool save_terrain(const Terrain& terrain, const std::string& path) {
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) return false;

    int32_t w = terrain.width_;
    int32_t h = terrain.height_;
    float cs = static_cast<float>(terrain.cell_size_);
    file.write(reinterpret_cast<const char*>(&w), 4);
    file.write(reinterpret_cast<const char*>(&h), 4);
    file.write(reinterpret_cast<const char*>(&cs), 4);

    file.write(reinterpret_cast<const char*>(terrain.heights_.data()),
               static_cast<std::streamsize>(terrain.heights_.size() * sizeof(float)));
    file.write(reinterpret_cast<const char*>(terrain.walkable_.data()),
               static_cast<std::streamsize>(terrain.walkable_.size()));
    file.write(reinterpret_cast<const char*>(terrain.cliff_level_.data()),
               static_cast<std::streamsize>(terrain.cliff_level_.size()));
    file.write(reinterpret_cast<const char*>(terrain.terrain_type_.data()),
               static_cast<std::streamsize>(terrain.terrain_type_.size()));
    file.write(reinterpret_cast<const char*>(terrain.water_.data()),
               static_cast<std::streamsize>(terrain.water_.size() * sizeof(float)));

    return file.good();
}

bool load_terrain(Terrain& terrain, const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        auto log = get_logger();
        log->error("Cannot open terrain file: {}", path);
        return false;
    }

    int32_t w, h;
    float cs;
    file.read(reinterpret_cast<char*>(&w), 4);
    file.read(reinterpret_cast<char*>(&h), 4);
    file.read(reinterpret_cast<char*>(&cs), 4);

    if (!file.good()) return false;

    terrain.width_ = w;
    terrain.height_ = h;
    terrain.cell_size_ = static_cast<double>(cs);

    size_t count = static_cast<size_t>(terrain.width_ * terrain.height_);
    terrain.heights_.resize(count);
    terrain.walkable_.resize(count);
    terrain.cliff_level_.resize(count);
    terrain.terrain_type_.resize(count);

    file.read(reinterpret_cast<char*>(terrain.heights_.data()),
              static_cast<std::streamsize>(count * sizeof(float)));
    file.read(reinterpret_cast<char*>(terrain.walkable_.data()),
              static_cast<std::streamsize>(count));
    file.read(reinterpret_cast<char*>(terrain.cliff_level_.data()),
              static_cast<std::streamsize>(count));
    file.read(reinterpret_cast<char*>(terrain.terrain_type_.data()),
              static_cast<std::streamsize>(count));
    terrain.water_.assign(count, Terrain::WATER_OFF);
    std::vector<float> water(count);
    file.read(reinterpret_cast<char*>(water.data()),
              static_cast<std::streamsize>(count * sizeof(float)));
    if (file.gcount() == static_cast<std::streamsize>(count * sizeof(float))) {
        terrain.water_ = std::move(water);
    } else {
        file.clear();
    }

    terrain.mark_dirty();
    return file.good();
}

}  // namespace robcraft::engine::world
