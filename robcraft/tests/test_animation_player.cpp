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
#include <vector>

#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/renderer/animation_player.hpp"
#include "robcraft/renderer/gltf_loader.hpp"

using namespace robcraft::engine::math;
using namespace robcraft::renderer;

using Catch::Approx;

TEST_CASE("sample_animation interpolates TRS", "[animation_player]") {
    robcraft::renderer::GltfAnimation clip;
    clip.name = "Test";
    clip.duration = 1.0f;
    robcraft::renderer::GltfAnimation::Track t;
    t.node = 0;
    t.path = robcraft::renderer::GltfAnimation::Translation;
    t.times = {0.0f, 1.0f};
    t.values = {0, 0, 0, 2, 4, 6};  // t0=(0,0,0) t1=(2,4,6)
    clip.tracks.push_back(t);
    robcraft::renderer::GltfAnimation::Track r;
    r.node = 0;
    r.path = robcraft::renderer::GltfAnimation::Rotation;
    r.times = {0.0f, 1.0f};
    r.values = {0, 0, 0, 1, 0, 0, 0, 1};  // glTF [x,y,z,w] = identity
    clip.tracks.push_back(r);

    robcraft::renderer::GltfSkin skin;
    skin.joint_nodes = {0};
    skin.parent = {-1};
    skin.inverse_bind.push_back(robcraft::engine::math::Mat4::from_position_rotation(
        robcraft::engine::math::Vec3(0, 0, 0), robcraft::engine::math::Quaternion::identity()));
    skin.node_local.push_back(robcraft::engine::math::Mat4::from_position_rotation(
        robcraft::engine::math::Vec3(0, 0, 0), robcraft::engine::math::Quaternion::identity()));
    skin.valid = true;

    auto mats = robcraft::renderer::sample_animation(skin, clip, 0.5f);
    REQUIRE(mats.size() == 1);
    // At t=0.5, translation = (1, 2, 3). Mat4 column-major: X→[12], Y→[13], Z→[14].
    REQUIRE(mats[0].ptr()[12] == Approx(1.0f));
    REQUIRE(mats[0].ptr()[13] == Approx(2.0f));
    REQUIRE(mats[0].ptr()[14] == Approx(3.0f));
}

TEST_CASE("sample_animation applies the frame conjugation", "[animation_player]") {
    robcraft::renderer::GltfSkin skin;
    skin.center = robcraft::engine::math::Vec3(1.0, 2.0, 0.0);
    skin.scale = 2.0;
    skin.joint_nodes = {0};
    skin.parent = {-1};
    // IBM_raw = identity; the loader stores inverse_bind pre-multiplied by the
    // normalization M = Scale(scale)·Translate(-center), so stored ib = M·I = M.
    robcraft::engine::math::Mat4 M =
        robcraft::engine::math::Mat4::scale_matrix(robcraft::engine::math::Vec3(2.0f, 2.0f, 2.0f)) *
        robcraft::engine::math::Mat4::from_position_rotation(
            robcraft::engine::math::Vec3(-1.0f, -2.0f, 0.0f),
            robcraft::engine::math::Quaternion::identity());
    skin.inverse_bind.push_back(M);
    // node_local bind: some offset, e.g. translate (0,1,0).
    skin.node_local.push_back(robcraft::engine::math::Mat4::from_position_rotation(
        robcraft::engine::math::Vec3(0.0, 1.0, 0.0),
        robcraft::engine::math::Quaternion::identity()));
    skin.valid = true;

    robcraft::renderer::GltfAnimation clip;
    clip.duration = 1.0f;
    clip.name = "T";
    robcraft::renderer::GltfAnimation::Track tr;
    tr.node = 0;
    tr.path = robcraft::renderer::GltfAnimation::Translation;
    tr.times = {0.0f, 1.0f};
    tr.values = {0, 1, 0, 0, 1, 0};  // constant translation (0,1,0)
    clip.tracks.push_back(tr);

    auto mats = robcraft::renderer::sample_animation(skin, clip, 0.0f);
    REQUIRE(mats.size() == 1);
    // Conjugation: J = M·(W·IBM_raw)·M⁻¹. With ib = M·I = M and W = Translate(0,1,0):
    //   J = M·W·M⁻¹ maps v -> M·(M⁻¹(v) + (0,1,0)) = v + 2·(0,1,0) = v + (0,2,0),
    // so the translation column is (0,2,0) — not the naive W·ib = (-2,-3,0).
    REQUIRE(mats[0].ptr()[12] == Approx(0.0f));
    REQUIRE(mats[0].ptr()[13] == Approx(2.0f));
    REQUIRE(mats[0].ptr()[14] == Approx(0.0f));
}
