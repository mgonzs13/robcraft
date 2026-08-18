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

#include "robcraft/renderer/model_paths.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/math/vec3.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/model.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

namespace {
const std::vector<PlacementSpec> kPlacementSpecs = {
    // Primitives (empty model_path)
    {"wall", "", Vec3(1.0, 3.0, 1.0), 0.5f, false, true},
    {"floor", "", Vec3(1.0, 0.01, 1.0), 0.5f, false, false},
    // Robots (mechs) — skinned glTF; "robot_" prefix keeps existing names working
    {"robot_george", "assets/models/mech/gltf/George.gltf", Vec3(1.11, 1.11, 1.11), 0.5f},
    {"robot_leela", "assets/models/mech/gltf/Leela.gltf", Vec3(1.00, 1.00, 1.00), 0.5f},
    {"robot_mike", "assets/models/mech/gltf/Mike.gltf", Vec3(1.60, 1.60, 1.60), 0.5f},
    {"robot_stan", "assets/models/mech/gltf/Stan.gltf", Vec3(1.325, 1.325, 1.325), 0.5f},
    // Animals — skinned glTF (Idle animation), blocking solid placeables
    {"cow", "assets/models/animals/gltf/Cow.gltf", Vec3(2.50, 2.50, 2.50), 0.5f, false, true},
    {"horse", "assets/models/animals/gltf/Horse.gltf", Vec3(2.60, 2.60, 2.60), 0.5f, false, true},
    {"llama", "assets/models/animals/gltf/Llama.gltf", Vec3(1.80, 1.80, 1.80), 0.5f, false, true},
    {"pig", "assets/models/animals/gltf/Pig.gltf", Vec3(1.50, 1.50, 1.50), 0.5f, false, true},
    {"pug", "assets/models/animals/gltf/Pug.gltf", Vec3(0.60, 0.60, 0.60), 0.5f, false, true},
    {"sheep", "assets/models/animals/gltf/Sheep.gltf", Vec3(1.30, 1.30, 1.30), 0.5f, false, true},
    {"zebra", "assets/models/animals/gltf/Zebra.gltf", Vec3(2.30, 2.30, 2.30), 0.5f, false, true},
    // Nature — Stylized Nature MegaKit trees and bushes
    {"tree", "assets/models/nature/CommonTree_1.obj", Vec3(7.00, 7.00, 7.00), 0.5f},
    {"tree_1", "assets/models/nature/CommonTree_1.obj", Vec3(7.00, 7.00, 7.00), 0.5f},
    {"tree_2", "assets/models/nature/CommonTree_2.obj", Vec3(7.00, 7.00, 7.00), 0.5f},
    {"tree_3", "assets/models/nature/CommonTree_3.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"tree_4", "assets/models/nature/CommonTree_4.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"tree_5", "assets/models/nature/CommonTree_5.obj", Vec3(7.00, 7.00, 7.00), 0.5f},
    {"pine_1", "assets/models/nature/Pine_1.obj", Vec3(7.00, 7.00, 7.00), 0.5f},
    {"pine_2", "assets/models/nature/Pine_2.obj", Vec3(7.00, 7.00, 7.00), 0.5f},
    {"pine_3", "assets/models/nature/Pine_3.obj", Vec3(7.00, 7.00, 7.00), 0.5f},
    {"pine_4", "assets/models/nature/Pine_4.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"pine_5", "assets/models/nature/Pine_5.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"twisted_1", "assets/models/nature/TwistedTree_1.obj", Vec3(9.00, 9.00, 9.00), 0.5f},
    {"twisted_2", "assets/models/nature/TwistedTree_2.obj", Vec3(9.00, 9.00, 9.00), 0.5f},
    {"twisted_3", "assets/models/nature/TwistedTree_3.obj", Vec3(9.00, 9.00, 9.00), 0.5f},
    {"twisted_4", "assets/models/nature/TwistedTree_4.obj", Vec3(9.00, 9.00, 9.00), 0.5f},
    {"twisted_5", "assets/models/nature/TwistedTree_5.obj", Vec3(9.00, 9.00, 9.00), 0.5f},
    {"dead_1", "assets/models/nature/DeadTree_1.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"dead_2", "assets/models/nature/DeadTree_2.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"dead_3", "assets/models/nature/DeadTree_3.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"dead_4", "assets/models/nature/DeadTree_4.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"dead_5", "assets/models/nature/DeadTree_5.obj", Vec3(8.00, 8.00, 8.00), 0.5f},
    {"bush", "assets/models/nature/Bush_Common.obj", Vec3(1.80, 1.80, 1.80), 0.5f},
    {"bush_2", "assets/models/nature/Bush_Common_Flowers.obj", Vec3(1.80, 1.80, 1.80), 0.5f},
    // Nature — rocks (textured via MTL map_Kd)
    {"rock_1", "assets/models/space/Rock_1.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"rock_2", "assets/models/space/Rock_2.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"rock_3", "assets/models/space/Rock_3.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"rock_4", "assets/models/space/Rock_4.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"rock_large_1", "assets/models/space/Rock_Large_1.obj", Vec3(3.00, 3.00, 3.00), 0.4f},
    {"rock_large_2", "assets/models/space/Rock_Large_2.obj", Vec3(3.00, 3.00, 3.00), 0.4f},
    {"rock_large_3", "assets/models/space/Rock_Large_3.obj", Vec3(3.00, 3.00, 3.00), 0.4f},
    // Furniture (sit on the floor via real bounds)
    {"bed", "assets/models/interior/Bed_Single.obj", Vec3(2.00, 2.00, 2.00), 0.5f, false, true},
    {"chair", "assets/models/interior/Chair_1.obj", Vec3(0.90, 0.90, 0.90), 0.5f, false, true},
    {"couch", "assets/models/interior/Couch_Large1.obj", Vec3(2.00, 2.00, 2.00), 0.5f, false, true},
    {"light_floor", "assets/models/interior/Light_Floor1.obj", Vec3(1.70, 1.70, 1.70), 0.5f, false,
     true},
    {"shelf", "assets/models/interior/Shelf_1.obj", Vec3(2.00, 2.00, 2.00), 0.5f, false, true},
    {"table", "assets/models/interior/Table_RoundLarge.obj", Vec3(1.80, 1.80, 1.80), 0.5f, false,
     true},
    // Space — moon rocks, base (multi-tile), dome, solar
    {"moon_rock", "assets/models/space/Rock_1.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"moon_rock_2", "assets/models/space/Rock_2.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"moon_rock_3", "assets/models/space/Rock_3.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"moon_rock_large", "assets/models/space/Rock_Large_1.obj", Vec3(3.00, 3.00, 3.00), 0.4f},
    {"space_base", "assets/models/space/Base_Large.obj", Vec3(12.00, 12.00, 12.00), 0.5f, true,
     true},
    {"geodesic_dome", "assets/models/space/GeodesicDome.obj", Vec3(8.00, 8.00, 8.00), 0.5f, false,
     true},
    {"solar_panel", "assets/models/space/SolarPanel_Ground.obj", Vec3(2.50, 2.50, 2.50), 0.3f},
    // Legacy aliases so existing worlds/entities still resolve.
    {"rock", "assets/models/space/Rock_2.obj", Vec3(1.50, 1.50, 1.50), 0.5f},
    {"base", "assets/models/space/Base_Large.obj", Vec3(12.00, 12.00, 12.00), 0.5f},
    {"box", "assets/models/space/Pickup_Crate.obj", Vec3(1.00, 1.00, 1.00), 0.5f},
    {"crate", "assets/models/space/Pickup_Crate.obj", Vec3(1.00, 1.00, 1.00), 0.5f},
    {"cone", "assets/models/space/Pickup_Sphere.obj", Vec3(0.90, 0.90, 0.90), 0.5f},
};

// Legacy aliases not represented as placement specs (name -> model path).
const std::vector<std::pair<std::string, std::string>> kLegacyAliases = {
    {"robot", "assets/models/mech/gltf/Mike.gltf"},
};
}  // namespace

std::string model_path_for_prefix(const std::string& prefix) {
    for (const auto& spec : kPlacementSpecs) {
        if (spec.name_prefix == prefix && !spec.model_path.empty()) return spec.model_path;
    }
    for (const auto& alias : kLegacyAliases) {
        if (alias.first == prefix) return alias.second;
    }
    return "assets/models/box/box.obj";
}

std::string mesh_label_for_name(const std::string& name) {
    // Ordered prefix -> .world mesh label, copied from the world serializer's
    // original chain. Prefix order matters (tree_5 before tree, moon_rock
    // before rock, bush_2 before bush, pine_ before pine-family names).
    static const std::vector<std::pair<std::string, std::string>> kMeshLabels = {
        {"wall", "wall"},
        {"box", "box"},
        {"floor", "floor"},
        {"pine_", "pine"},
        {"twisted_", "twisted"},
        {"dead_", "dead"},
        {"tree_5", "tree_5"},
        {"tree_4", "tree_4"},
        {"tree_3", "tree_3"},
        {"tree_2", "tree_2"},
        {"tree_1", "tree_1"},
        {"tree", "tree"},
        {"bush_2", "bush_2"},
        {"bush", "bush"},
        {"space_base", "space_base"},
        {"geodesic_dome", "geodesic_dome"},
        {"moon_rock", "moon_rock"},
        {"solar_panel", "solar_panel"},
        {"rock", "rock"},
        {"bed", "bed"},
        {"chair", "chair"},
        {"couch", "couch"},
        {"light_floor", "light_floor"},
        {"shelf", "shelf"},
        {"table", "table"},
        {"cow", "cow"},
        {"horse", "horse"},
        {"llama", "llama"},
        {"pig", "pig"},
        {"pug", "pug"},
        {"sheep", "sheep"},
        {"zebra", "zebra"},
        {"cone", "cone"},
    };
    for (const auto& [prefix, label] : kMeshLabels) {
        if (name.rfind(prefix, 0) == 0) return label;
    }
    return "cube";
}

const PlacementSpec* placement_spec_for_name(const std::string& name) {
    const PlacementSpec* best = nullptr;
    size_t best_len = 0;
    for (const auto& s : kPlacementSpecs) {
        if (name.rfind(s.name_prefix, 0) == 0 && s.name_prefix.size() > best_len) {
            best = &s;
            best_len = s.name_prefix.size();
        }
    }
    return best;
}

const PlacementSpec* placement_spec_for_prefix(const std::string& prefix) {
    for (const auto& s : kPlacementSpecs) {
        if (s.name_prefix == prefix) return &s;
    }
    return nullptr;
}

std::string draw_model_path_for_name(const std::string& name) {
    const PlacementSpec* spec = placement_spec_for_name(name);
    if (spec) return spec->model_path;
    return model_path_for_prefix(name);
}

float placement_ground_offset(const PlacementSpec* spec, const Model* model, const Vec3& scale) {
    if (model && model->valid()) {
        // Real bounds: place the model's lowest vertex exactly on the ground.
        return static_cast<float>(-model->bounds_min().y * scale.y);
    }
    if (spec) return spec->ground_frac * scale.y;
    return 0.5f * scale.y;
}

std::pair<int, int> placement_footprint_cells(const PlacementSpec* spec, double cell_size) {
    if (!spec || !spec->multi_tile || cell_size <= 0.0) return {1, 1};
    int w = static_cast<int>(std::ceil(spec->base_scale.x / cell_size));
    int d = static_cast<int>(std::ceil(spec->base_scale.z / cell_size));
    return {std::max(w, 1), std::max(d, 1)};
}

Vec3 collider_half_extents(const Model* model, const Vec3& scale) {
    if (model && model->valid()) {
        Vec3 ext = model->bounds_max() - model->bounds_min();
        return Vec3(ext.x * scale.x * 0.5, ext.y * scale.y * 0.5, ext.z * scale.z * 0.5);
    }
    return scale * 0.5;
}

void refit_world_colliders(World& world, ModelCache& cache) {
    auto* cs = world.store<BoxCollider>();
    auto* ts = world.store<Transform3D>();
    auto* ns = world.store<Name>();
    if (!cs || !ts || !ns) return;
    for (auto& [e, col] : *cs) {
        auto* tf = ts->get(e);
        auto* nm = ns->get(e);
        if (!tf || !nm) continue;
        std::string path = draw_model_path_for_name(nm->value);
        if (path.empty()) continue;
        auto model = cache.get(path);
        if (!model || !model->valid()) continue;
        col.half_extents = collider_half_extents(model.get(), tf->scale);
    }
}

}  // namespace robcraft::renderer
