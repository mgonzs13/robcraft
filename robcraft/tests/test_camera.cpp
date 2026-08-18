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

#include "robcraft/renderer/fbo.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"

using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;

using Catch::Approx;

TEST_CASE("CameraSensor default configuration", "[camera]") {
    robcraft::sensors::camera::CameraSensor cam;

    REQUIRE(cam.width == 640);
    REQUIRE(cam.height == 480);
    REQUIRE(cam.fov_deg == Approx(60.0));
    REQUIRE(cam.update_rate == Approx(30.0));
    REQUIRE(cam.image_data.size() == 640 * 480 * 3);
}

TEST_CASE("CameraSensor rebuild resizes image buffer", "[camera]") {
    robcraft::sensors::camera::CameraSensor cam;
    cam.width = 160;
    cam.height = 120;
    cam.rebuild();

    REQUIRE(cam.image_data.size() == 160 * 120 * 3);
}

TEST_CASE("CameraSensor time accumulator", "[camera]") {
    robcraft::sensors::camera::CameraSensor cam;
    cam.update_rate = 10.0;
    cam.time_since_update = 0.0;

    cam.time_since_update += 0.05;
    REQUIRE(cam.time_since_update < 1.0 / cam.update_rate);

    cam.time_since_update += 0.06;
    REQUIRE(cam.time_since_update >= 1.0 / cam.update_rate);
}

TEST_CASE("flip_vertical_rgb mirrors rows top-to-bottom", "[camera]") {
    // 2x2 RGB: top row red, bottom row green.
    std::vector<uint8_t> img = {
        255, 0,   0, 255, 0,   0,  //
        0,   255, 0, 0,   255, 0,  //
    };
    robcraft::renderer::flip_vertical_rgb(img, 2, 2);

    std::vector<uint8_t> expected = {
        0,   255, 0, 0,   255, 0,  //
        255, 0,   0, 255, 0,   0,  //
    };
    REQUIRE(img == expected);
}

TEST_CASE("flip_vertical_rgb handles odd height", "[camera]") {
    // 1x3 RGB: rows red, green, blue.
    std::vector<uint8_t> img = {
        255, 0,   0,    //
        0,   255, 0,    //
        0,   0,   255,  //
    };
    robcraft::renderer::flip_vertical_rgb(img, 1, 3);

    std::vector<uint8_t> expected = {
        0,   0,   255,  //
        0,   255, 0,    //
        255, 0,   0,    //
    };
    REQUIRE(img == expected);
}
