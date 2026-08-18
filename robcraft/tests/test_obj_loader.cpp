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

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "robcraft/renderer/obj_loader.hpp"

using namespace robcraft::renderer;

using Catch::Approx;

TEST_CASE("OBJ loader parses positions, normals, UVs, faces", "[obj_loader]") {
    const std::string obj = R"(
# quad
v 0 0 0
v 1 0 0
v 1 0 1
v 0 0 1
vn 0 1 0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
)";
    robcraft::renderer::ObjData data;
    REQUIRE(robcraft::renderer::load_obj_from_memory_with_mtl(obj, "", data));
    REQUIRE(data.vertices.size() == 6);  // 2 triangles * 3 corners
    // Unit normalization centers at origin: world (0,0,0) → (-0.5, 0, -0.5) for a 1x1 quad.
    REQUIRE(data.vertices[0].x == Approx(-0.5f));
    REQUIRE(data.vertices[0].z == Approx(-0.5f));
    REQUIRE(data.vertices[2].u == Approx(1.0f));
    REQUIRE(data.vertices[2].v == Approx(1.0f));
}

TEST_CASE("OBJ loader handles vertex colors and unit normalization", "[obj_loader]") {
    const std::string obj = R"(
v 0 0 0 1 0 0
v 2 0 0 1 0 0
v 0 2 0 1 0 0
f 1 2 3
)";
    robcraft::renderer::ObjData data;
    REQUIRE(robcraft::renderer::load_obj_from_memory_with_mtl(obj, "", data));
    REQUIRE(data.vertices.size() == 3);
    REQUIRE(data.vertices[0].r == Approx(1.0));
    REQUIRE(data.vertices[0].g == Approx(0.0));
    // Unit normalization: max extent becomes 1.0
    float maxe = 0.0f;
    for (const auto& v : data.vertices) maxe = std::max(maxe, std::max(v.x, std::max(v.y, v.z)));
    REQUIRE(maxe == Approx(0.5f));  // half-extent after centering+scaling to unit AABB
}

TEST_CASE("OBJ loader rejects invalid face indices", "[obj_loader]") {
    const std::string obj = R"(
v 0 0 0
v 1 0 0
v 1 0 1
f 1 2 9      # index 9 out of range
f 1 2 3      # valid
)";
    robcraft::renderer::ObjData data;
    REQUIRE(robcraft::renderer::load_obj_from_memory_with_mtl(obj, "", data));
    REQUIRE(data.vertices.size() == 3);  // only the valid face emitted
}

TEST_CASE("OBJ loader tolerates malformed face tokens", "[obj_loader]") {
    const std::string obj = R"(
v 0 0 0
v 1 0 0
v 1 0 1
f 1/a/2 2 3   # malformed first corner
f 1 2 3       # valid
)";
    robcraft::renderer::ObjData data;
    REQUIRE(robcraft::renderer::load_obj_from_memory_with_mtl(obj, "", data));
    REQUIRE(data.vertices.size() == 3);  // malformed face skipped, valid one kept
}

TEST_CASE("OBJ loader bakes MTL Kd colors via usemtl", "[obj_loader]") {
    const std::string obj = R"(
v 0 0 0
v 1 0 0
v 0 1 0
usemtl Red
f 1 2 3
v 0 0 1
v 1 0 1
v 0 1 1
usemtl Blue
f 4 5 6
)";
    const std::string mtl = R"(
newmtl Red
Kd 1 0 0
newmtl Blue
Kd 0 0 1
)";
    robcraft::renderer::ObjData data;
    REQUIRE(robcraft::renderer::load_obj_from_memory_with_mtl(obj, mtl, data));
    REQUIRE(data.vertices.size() == 6);
    REQUIRE(data.vertex_material.size() == 6);
    REQUIRE(data.materials.size() == 2);
    // First triangle's vertices are Red (material index 0) with Kd baked in
    REQUIRE(data.vertex_material[0] == 0);
    REQUIRE(data.vertices[0].r == Approx(1.0f));
    REQUIRE(data.vertices[0].g == Approx(0.0f));
    REQUIRE(data.vertices[0].b == Approx(0.0f));
    // Second triangle's vertices are Blue (material index 1)
    REQUIRE(data.vertex_material[3] == 1);
    REQUIRE(data.vertices[3].r == Approx(0.0f));
    REQUIRE(data.vertices[3].b == Approx(1.0f));
}

TEST_CASE("OBJ loader prefers explicit vertex colors over MTL Kd", "[obj_loader]") {
    const std::string obj = R"(
v 0 0 0 0 1 0
v 1 0 0 0 1 0
v 0 1 0 0 1 0
usemtl Red
f 1 2 3
)";
    const std::string mtl = "newmtl Red\nKd 1 0 0\n";
    robcraft::renderer::ObjData data;
    REQUIRE(robcraft::renderer::load_obj_from_memory_with_mtl(obj, mtl, data));
    REQUIRE(data.vertices.size() == 3);
    REQUIRE(data.vertex_material[0] == 0);
    REQUIRE(data.vertices[0].r == Approx(0.0f));  // explicit green wins over red Kd
    REQUIRE(data.vertices[0].g == Approx(1.0f));
}
