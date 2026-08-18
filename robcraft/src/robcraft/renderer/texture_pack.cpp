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

#include "robcraft/renderer/texture_pack.hpp"

#include <filesystem>
#include <string>
#include <vector>

#include "robcraft/engine/core/data_path.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::core;

void TexturePack::load(int new_size) {
    this->terrain_albedo.destroy();
    this->terrain_normal.destroy();
    this->wall_albedo.destroy();
    this->wall_normal.destroy();
    this->floor_albedo.destroy();
    this->floor_normal.destroy();

    auto tex_dir = [](int size) {
        return robcraft::engine::core::resolve_data_path("assets/textures/terrain/" +
                                                         std::to_string(size) + "/");
    };
    std::vector<std::string> albedo_layers, normal_layers;
    std::string layer_dir;
    int resolved = new_size;
    for (int size : {new_size, 512, 256}) {
        std::string d = tex_dir(size);
        if (std::filesystem::exists(d)) {
            layer_dir = d;
            resolved = size;
            break;
        }
    }
    if (!layer_dir.empty()) {
        const std::vector<std::string> types = {"grass", "dirt", "rock", "sand", "snow"};
        for (const auto& t : types) {
            albedo_layers.push_back(layer_dir + t + "_diffuse.png");
            normal_layers.push_back(layer_dir + t + "_normal.png");
        }
    }
    this->terrain_albedo = Texture::create_2d_array(albedo_layers);
    this->terrain_normal = Texture::create_2d_array(normal_layers);
    this->use_splat = this->terrain_albedo.valid() && this->terrain_normal.valid();
    this->size = resolved;

    auto btex_dir = [](int size) {
        return robcraft::engine::core::resolve_data_path("assets/textures/building/" +
                                                         std::to_string(size) + "/");
    };
    std::string bdir;
    for (int size : {resolved, 512, 256}) {
        if (std::filesystem::exists(btex_dir(size))) {
            bdir = btex_dir(size);
            break;
        }
    }
    this->wall_albedo = Texture::create_2d(bdir + "wall_diffuse.png");
    this->wall_normal = Texture::create_2d(bdir + "wall_normal.png");
    this->floor_albedo = Texture::create_2d(bdir + "floor_diffuse.png");
    this->floor_normal = Texture::create_2d(bdir + "floor_normal.png");
}

void TexturePack::destroy() {
    this->terrain_albedo.destroy();
    this->terrain_normal.destroy();
    this->wall_albedo.destroy();
    this->wall_normal.destroy();
    this->floor_albedo.destroy();
    this->floor_normal.destroy();
}

}  // namespace robcraft::renderer
