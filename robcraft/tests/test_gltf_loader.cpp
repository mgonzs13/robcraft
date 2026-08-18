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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "robcraft/renderer/gltf_loader.hpp"

using namespace robcraft::renderer;

using Catch::Approx;

TEST_CASE("gltf loader decodes accessors and builds meshes", "[gltf_loader]") {
    // Minimal glTF 2.0: one triangle, POSITION only, no skin/anim yet.
    const std::string json_text = R"({
  "asset": {"version": "2.0"},
  "scene": 0,
  "scenes": [{"nodes": [0]}],
  "nodes": [{"mesh": 0}],
  "meshes": [{"primitives": [{
    "attributes": {"POSITION": 0},
    "indices": 1
  }]}],
  "accessors": [
    {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
    {"bufferView": 1, "componentType": 5123, "count": 3, "type": "SCALAR"}
  ],
  "bufferViews": [
    {"buffer": 0, "byteOffset": 0, "byteLength": 36},
    {"buffer": 0, "byteOffset": 36, "byteLength": 6}
  ],
  "buffers": [{"byteLength": 42, "uri": "data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAABAAIA"}]
})";
    // base64: 3 floats (0,0,0)(1,0,0)(0,1,0) LE = 36 bytes + 3 u16 indices 0,1,2 = 6 bytes (42
    // total)
    robcraft::renderer::GltfModelData data;
    REQUIRE(robcraft::renderer::load_gltf_from_memory(json_text, data));
    REQUIRE(data.meshes.size() == 1);
    REQUIRE(data.meshes[0].vertices.size() == 3);
    REQUIRE(data.meshes[0].indices.size() == 3);
    // Unit normalization centers at origin: (0,0,0) → (-0.5, -0.5, 0) for a 1x1 triangle.
    REQUIRE(data.meshes[0].vertices[0].x == Approx(-0.5f));
    REQUIRE(data.meshes[0].vertices[1].x == Approx(0.5f));
    REQUIRE(data.meshes[0].vertices[2].y == Approx(0.5f));
    // Combined transform is carried on the model: center (0.5, 0.5, 0), scale 1.0.
    REQUIRE(data.center.x == Approx(0.5));
    REQUIRE(data.center.y == Approx(0.5));
    REQUIRE(data.center.z == Approx(0.0));
    REQUIRE(data.scale == Approx(1.0));
}

TEST_CASE("gltf loader decodes ubyte JOINTS_0 accessors", "[gltf_loader]") {
    // Same triangle but with a ubyte VEC4 JOINTS_0 accessor (the mech pack layout).
    const std::string json_text = R"({
  "asset": {"version": "2.0"},
  "scene": 0,
  "scenes": [{"nodes": [0]}],
  "nodes": [{"mesh": 0}],
  "meshes": [{"primitives": [{
    "attributes": {"POSITION": 0, "JOINTS_0": 1},
    "indices": 2
  }]}],
  "accessors": [
    {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
    {"bufferView": 1, "componentType": 5121, "count": 3, "type": "VEC4"},
    {"bufferView": 2, "componentType": 5123, "count": 3, "type": "SCALAR"}
  ],
  "bufferViews": [
    {"buffer": 0, "byteOffset": 0, "byteLength": 36},
    {"buffer": 0, "byteOffset": 36, "byteLength": 12},
    {"buffer": 0, "byteOffset": 48, "byteLength": 6}
  ],
  "buffers": [{"byteLength": 54, "uri": "data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAECAwECAwQCAwQFAAABAAIA"}]
})";
    robcraft::renderer::GltfModelData data;
    REQUIRE(robcraft::renderer::load_gltf_from_memory(json_text, data));
    REQUIRE(data.meshes.size() == 1);
    REQUIRE(data.meshes[0].vertices.size() == 3);
    REQUIRE(data.meshes[0].joint_indices.size() == 12);  // 4 per vertex
    REQUIRE(data.meshes[0].joint_indices[0] == 0);
    REQUIRE(data.meshes[0].joint_indices[1] == 1);
    REQUIRE(data.meshes[0].joint_indices[2] == 2);
    REQUIRE(data.meshes[0].joint_indices[3] == 3);
    REQUIRE(data.meshes[0].joint_indices[4] == 1);  // vertex 1
    REQUIRE(data.meshes[0].joint_indices[8] == 2);  // vertex 2
}

TEST_CASE("gltf loader decodes ubyte index accessors", "[gltf_loader]") {
    // Triangle with a ubyte (5121) SCALAR index accessor.
    const std::string json_text = R"({
  "asset": {"version": "2.0"},
  "scene": 0,
  "scenes": [{"nodes": [0]}],
  "nodes": [{"mesh": 0}],
  "meshes": [{"primitives": [{
    "attributes": {"POSITION": 0},
    "indices": 1
  }]}],
  "accessors": [
    {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
    {"bufferView": 1, "componentType": 5121, "count": 3, "type": "SCALAR"}
  ],
  "bufferViews": [
    {"buffer": 0, "byteOffset": 0, "byteLength": 36},
    {"buffer": 0, "byteOffset": 36, "byteLength": 3}
  ],
  "buffers": [{"byteLength": 39, "uri": "data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAEC"}]
})";
    robcraft::renderer::GltfModelData data;
    REQUIRE(robcraft::renderer::load_gltf_from_memory(json_text, data));
    REQUIRE(data.meshes.size() == 1);
    REQUIRE(data.meshes[0].indices.size() == 3);
    REQUIRE(data.meshes[0].indices[0] == 0);
    REQUIRE(data.meshes[0].indices[1] == 1);
    REQUIRE(data.meshes[0].indices[2] == 2);
}

TEST_CASE("gltf loader parses skin and animation", "[gltf_loader]") {
    // 3 pos floats(36) + 3 u16 idx(6) + 1 MAT4 identity(64) + 2 times(8)
    // + 2 VEC3(24) = 138 bytes. MAT4 is column-major float32 identity;
    // times are 0.0, 1.0; translations are (0,0,0) and (1,2,3).
    const std::string json_text = R"({
  "asset": {"version": "2.0"},
  "scene": 0,
  "scenes": [{"nodes": [0]}],
  "nodes": [
    {"name": "Root", "children": [1]},
    {"name": "Joint", "translation": [0, 1, 0]}
  ],
  "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
  "accessors": [
    {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
    {"bufferView": 1, "componentType": 5123, "count": 3, "type": "SCALAR"},
    {"bufferView": 2, "componentType": 5126, "count": 1, "type": "MAT4"},
    {"bufferView": 3, "componentType": 5126, "count": 2, "type": "SCALAR"},
    {"bufferView": 4, "componentType": 5126, "count": 2, "type": "VEC3"}
  ],
  "bufferViews": [
    {"buffer": 0, "byteOffset": 0, "byteLength": 36},
    {"buffer": 0, "byteOffset": 36, "byteLength": 6},
    {"buffer": 0, "byteOffset": 42, "byteLength": 64},
    {"buffer": 0, "byteOffset": 106, "byteLength": 8},
    {"buffer": 0, "byteOffset": 114, "byteLength": 24}
  ],
  "buffers": [{"byteLength": 138, "uri": "data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAABAAIAAACAPwAAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAAAAAAAAAAAAgD8AAAAAAAAAAAAAAAAAAAAAAACAPwAAAAAAAIA/AAAAAAAAAAAAAAAAAACAPwAAAEAAAEBA"}],
  "skins": [{"joints": [1], "inverseBindMatrices": 2}],
  "animations": [{"name": "Bob", "channels": [
     {"sampler": 0, "target": {"node": 1, "path": "translation"}}
  ], "samplers": [{"input": 3, "output": 4, "interpolation": "LINEAR"}]}]
})";
    robcraft::renderer::GltfModelData data;
    REQUIRE(robcraft::renderer::load_gltf_from_memory(json_text, data));
    REQUIRE(data.skin.valid);
    REQUIRE(data.skin.joint_nodes.size() == 1);
    REQUIRE(data.skin.joint_nodes[0] == 1);
    REQUIRE(data.skin.inverse_bind.size() == 1);
    REQUIRE(data.skin.parent.size() == 2);
    REQUIRE(data.skin.parent[0] == -1);  // Root has no parent
    REQUIRE(data.skin.parent[1] == 0);
    // Identity IBM composed with T = translate(-center) * scale(1) must equal
    // translate(-center); center of the 1x1 triangle is (0.5, 0.5, 0).
    REQUIRE(data.skin.inverse_bind[0].ptr()[12] == Approx(-0.5f));
    REQUIRE(data.skin.inverse_bind[0].ptr()[13] == Approx(-0.5f));
    REQUIRE(data.skin.inverse_bind[0].ptr()[14] == Approx(0.0f));
    // node_local for the joint carries the bind translation [0, 1, 0].
    REQUIRE(data.skin.node_local.size() == 2);
    REQUIRE(data.skin.node_local[1].ptr()[13] == Approx(1.0f));

    REQUIRE(data.animations.size() == 1);
    REQUIRE(data.animations[0].name == "Bob");
    REQUIRE(data.animations[0].tracks.size() == 1);
    REQUIRE(data.animations[0].tracks[0].node == 1);
    REQUIRE(data.animations[0].tracks[0].path == robcraft::renderer::GltfAnimation::Translation);
    REQUIRE(data.animations[0].tracks[0].times.size() == 2);
    REQUIRE(data.animations[0].tracks[0].times[0] == Approx(0.0f));
    REQUIRE(data.animations[0].tracks[0].times[1] == Approx(1.0f));
    REQUIRE(data.animations[0].tracks[0].values.size() == 6);  // 2 keys × 3 floats
    REQUIRE(data.animations[0].tracks[0].values[0] == Approx(0.0f));
    REQUIRE(data.animations[0].tracks[0].values[4] == Approx(2.0f));
    REQUIRE(data.animations[0].duration == Approx(1.0f));
}
