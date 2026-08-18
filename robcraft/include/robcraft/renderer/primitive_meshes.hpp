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

#include "robcraft/renderer/mesh.hpp"

namespace robcraft::renderer {

/** @brief The procedural fallback meshes shared by the sim and the editor. */
struct PrimitiveMeshes {
    /** @brief Robots / boxes. */
    Mesh cube;
    /** @brief Walls and floors (flat plate reuse). */
    Mesh wall;
    /** @brief Rocks / cones (editor only). */
    Mesh pyramid;
    /** @brief Simple round tree. */
    Mesh tree;
    /** @brief Conical pine. */
    Mesh pine;
    /** @brief Round bush. */
    Mesh bush;

    /** @brief Destroys all meshes. */
    void destroy();
};

/** @brief Builds all shared primitive meshes. Requires a GL context.
 *  @return The built meshes. */
PrimitiveMeshes make_primitive_meshes();

}  // namespace robcraft::renderer
