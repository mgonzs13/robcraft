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

#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"

using namespace robcraft::sensors::depth_camera;

using Catch::Approx;

TEST_CASE("DepthCameraSensor default configuration", "[depth_camera]") {
    robcraft::sensors::depth_camera::DepthCameraSensor cam;
    REQUIRE(cam.update_rate == Approx(30.0));
}

TEST_CASE("DepthCameraSensor rebuild resizes depth buffer", "[depth_camera]") {
    robcraft::sensors::depth_camera::DepthCameraSensor cam;
    cam.width = 320;
    cam.height = 240;
    cam.rebuild();
    REQUIRE(cam.depth_data.size() == 320 * 240);
}

TEST_CASE("DepthCameraSensor initializes depth buffer to no-data", "[depth_camera]") {
    // Depth is initialized to 0 (invalid/no data) rather than the far plane, so
    // consumers never see phantom far-range readings before the first render.
    robcraft::sensors::depth_camera::DepthCameraSensor cam;
    REQUIRE(cam.depth_data.size() == static_cast<size_t>(cam.width) * cam.height);
    for (float z : cam.depth_data) REQUIRE(z == 0.0f);
}

TEST_CASE("linearize_depth maps near plane to near", "[depth_camera]") {
    REQUIRE(robcraft::sensors::depth_camera::linearize_depth(0.1f, 100.0f, 0.0f) ==
            Approx(0.1f).margin(1e-4f));
}

TEST_CASE("linearize_depth maps far plane to far", "[depth_camera]") {
    // Raw depth buffer value 1.0 == far plane.
    REQUIRE(robcraft::sensors::depth_camera::linearize_depth(0.1f, 100.0f, 1.0f) ==
            Approx(100.0f).margin(0.1f));
}

TEST_CASE("linearize_depth is monotonic and in range", "[depth_camera]") {
    float prev = 0.0f;
    for (int i = 0; i <= 100; ++i) {
        float z = robcraft::sensors::depth_camera::linearize_depth(0.1f, 100.0f, i / 100.0f);
        REQUIRE(z >= 0.1f);
        REQUIRE(z <= 100.0f);
        REQUIRE(z >= prev);
        prev = z;
    }
}

TEST_CASE("linearize_depth midpoint lies between near and far", "[depth_camera]") {
    float z = robcraft::sensors::depth_camera::linearize_depth(0.1f, 100.0f, 0.5f);
    REQUIRE(z > 0.1f);
    REQUIRE(z < 100.0f);
}
