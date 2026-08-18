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

#include "robcraft/renderer/primitive_meshes.hpp"

namespace robcraft::renderer {

PrimitiveMeshes make_primitive_meshes() {
    PrimitiveMeshes p;
    p.cube = Mesh::create_cube(1.0f, 0.2f, 0.6f, 0.9f);
    // Shared wall dims (was the editor's values; the main app used 0.5/0.45/0.4).
    p.wall = Mesh::create_cube(1.0f, 0.55f, 0.5f, 0.45f);
    p.pyramid = Mesh::create_pyramid(1.0f, 1.2f, 0.6f, 0.35f, 0.2f);
    p.tree = Mesh::create_simple_tree();
    p.pine = Mesh::create_pine();
    p.bush = Mesh::create_bush();
    return p;
}

void PrimitiveMeshes::destroy() {
    this->cube.destroy();
    this->wall.destroy();
    this->pyramid.destroy();
    this->tree.destroy();
    this->pine.destroy();
    this->bush.destroy();
}

}  // namespace robcraft::renderer
