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

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "robcraft/renderer/texture.hpp"

using namespace robcraft::renderer;

TEST_CASE("image_has_alpha detects transparency", "[texture]") {
    robcraft::renderer::Image opaque;
    opaque.width = 1;
    opaque.height = 1;
    opaque.channels = 4;
    opaque.pixels = {200, 100, 50, 255};
    REQUIRE_FALSE(robcraft::renderer::image_has_alpha(opaque));

    robcraft::renderer::Image transparent;
    transparent.width = 2;
    transparent.height = 1;
    transparent.channels = 4;
    transparent.pixels = {200, 100, 50, 255, 10, 20, 30, 0};
    REQUIRE(robcraft::renderer::image_has_alpha(transparent));

    robcraft::renderer::Image partial;
    partial.width = 2;
    partial.height = 1;
    partial.channels = 4;
    partial.pixels = {200, 100, 50, 255, 10, 20, 30, 128};
    REQUIRE(robcraft::renderer::image_has_alpha(partial));
}

TEST_CASE("generate_water_normal_image is deterministic and tiling", "[texture]") {
    auto a = robcraft::renderer::Texture::generate_water_normal_image();
    auto b = robcraft::renderer::Texture::generate_water_normal_image();
    REQUIRE(a.width == 256);
    REQUIRE(a.height == 256);
    REQUIRE(a.channels == 4);
    REQUIRE(a.pixels == b.pixels);  // deterministic
    REQUIRE(a.pixels.size() == static_cast<size_t>(256) * 256 * 4);

    // Normal points up on average: green channel (encoded +Y) is >= 0.5*255.
    // Ripple detail exists: pixels are not all identical.
    bool non_flat = false;
    for (int y = 0; y < 256; ++y) {
        for (int x = 0; x < 256; ++x) {
            size_t p = static_cast<size_t>(y * 256 + x) * 4;
            REQUIRE(a.pixels[p + 1] >= 128);
            if (a.pixels[p + 1] != a.pixels[1]) non_flat = true;
        }
    }
    REQUIRE(non_flat);

    // Tile seam is seamless: leftmost column equals rightmost column
    // (every wave completes an integer number of cycles per tile).
    for (int y = 0; y < 256; ++y) {
        size_t left = static_cast<size_t>(y * 256) * 4;
        size_t right = static_cast<size_t>(y * 256 + 255) * 4;
        for (int c = 0; c < 3; ++c) {
            REQUIRE(a.pixels[left + c] == a.pixels[right + c]);
        }
    }
}
