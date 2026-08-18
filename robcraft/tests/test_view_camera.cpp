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
#include <cmath>

#include "robcraft/renderer/camera.hpp"

using namespace robcraft::renderer;

using Catch::Approx;

namespace {

double dot(const robcraft::engine::math::Vec3& a, const robcraft::engine::math::Vec3& b) {
    return a.dot(b);
}

}  // namespace

TEST_CASE("Camera rotate changes facing without moving position", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(10.0, 5.0, 0.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    robcraft::engine::math::Vec3 before = cam.forward();
    robcraft::engine::math::Vec3 pos_before = cam.position();

    cam.rotate(0.5f, 0.0f);

    REQUIRE(dot(before, cam.forward()) < 1.0 - 1e-3);
    REQUIRE(cam.position() == pos_before);
}

TEST_CASE("Camera rotate clamps pitch", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(10.0, 5.0, 0.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    for (int i = 0; i < 100; ++i) cam.rotate(0.0f, 0.5f);
    REQUIRE(cam.forward().y > 0.99);

    for (int i = 0; i < 200; ++i) cam.rotate(0.0f, -0.5f);
    REQUIRE(cam.forward().y < -0.99);
}

TEST_CASE("Camera rotate after look_at does not jump", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(50.0, 40.0, 50.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    robcraft::engine::math::Vec3 facing = cam.forward();
    cam.rotate(0.0f, 0.0f);
    REQUIRE(dot(facing, cam.forward()) > 1.0 - 1e-4);

    robcraft::engine::math::Vec3 after = cam.forward();
    cam.rotate(0.5f, 0.0f);
    REQUIRE(dot(after, cam.forward()) > std::cos(0.5) - 1e-3);
}

TEST_CASE("Camera rotate keeps forward normalized", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    cam.look_at(robcraft::engine::math::Vec3(1.0, 1.0, 1.0));
    cam.rotate(1.2f, 0.4f);
    cam.rotate(-0.7f, 1.0f);

    robcraft::engine::math::Vec3 f = cam.forward();
    REQUIRE(std::sqrt(dot(f, f)) == Approx(1.0).margin(1e-4));
}

TEST_CASE("Camera has_orbit_target after set_orbit_target", "[view_camera]") {
    robcraft::renderer::Camera cam;
    REQUIRE(!cam.has_orbit_target());
    cam.set_orbit_target(robcraft::engine::math::Vec3(1.0, 2.0, 3.0));
    REQUIRE(cam.has_orbit_target());
}

TEST_CASE("Camera zoom by factor reduces distance and clamps", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    cam.set_orbit_target(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    double d0 = (cam.position() - cam.orbit_target()).length();
    cam.zoom_by_factor(0.5f);
    double d1 = (cam.position() - cam.orbit_target()).length();
    REQUIRE(d1 == Approx(d0 * 0.5));

    for (int i = 0; i < 100; ++i) cam.zoom_by_factor(0.1f);
    double dmin = (cam.position() - cam.orbit_target()).length();
    REQUIRE(dmin >= Approx(0.5).margin(1e-6));

    for (int i = 0; i < 100; ++i) cam.zoom_by_factor(10.0f);
    double dmax = (cam.position() - cam.orbit_target()).length();
    REQUIRE(dmax <= Approx(500.0).margin(1e-6));
}

TEST_CASE("Camera zoom by factor keeps facing", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    cam.set_orbit_target(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    cam.zoom_by_factor(0.5f);
    robcraft::engine::math::Vec3 toward = (cam.orbit_target() - cam.position()).normalized();
    REQUIRE(toward.dot(cam.forward()) == Approx(1.0).margin(1e-4));
}

TEST_CASE("Camera orbit keeps distance to target", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    cam.set_orbit_target(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    double dist = (cam.position() - cam.orbit_target()).length();
    cam.orbit(0.5f, 0.3f, static_cast<float>(dist));
    double d2 = (cam.position() - cam.orbit_target()).length();
    REQUIRE(d2 == Approx(dist).margin(1e-3));
}

TEST_CASE("Camera zoom by factor is a no-op before any target", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    cam.zoom_by_factor(0.5f);
    REQUIRE((cam.position() - robcraft::engine::math::Vec3(0.0, 10.0, 20.0)).length() ==
            Approx(0.0).margin(1e-9));
}

TEST_CASE("Camera orbit is a no-op before any target", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    robcraft::engine::math::Vec3 before = cam.position();
    cam.orbit(0.5f, 0.3f, 20.0f);
    REQUIRE(cam.position() == before);
}

TEST_CASE("Camera zoom by factor with factor > 1 increases distance", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    cam.set_orbit_target(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    double d0 = (cam.position() - cam.orbit_target()).length();
    cam.zoom_by_factor(2.0f);
    double d1 = (cam.position() - cam.orbit_target()).length();
    REQUIRE(d1 == Approx(d0 * 2.0));
}

TEST_CASE("Camera pan moves horizontally and keeps the orbit target", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    cam.set_orbit_target(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    robcraft::engine::math::Vec3 p0 = cam.position();
    robcraft::engine::math::Vec3 t0 = cam.orbit_target();
    cam.pan(10.0f, 0.0f);
    robcraft::engine::math::Vec3 delta = cam.position() - p0;
    REQUIRE(delta.y == Approx(0.0).margin(1e-9));
    REQUIRE(delta.length() == Approx(10.0));
    REQUIRE((cam.orbit_target() - t0 - delta).length() == Approx(0.0).margin(1e-6));
}

TEST_CASE("Camera move_world_up moves on the world Y axis regardless of orientation",
          "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));  // tilted view
    robcraft::engine::math::Vec3 p0 = cam.position();
    cam.move_world_up(5.0f);
    robcraft::engine::math::Vec3 delta = cam.position() - p0;
    REQUIRE(delta.x == Approx(0.0).margin(1e-9));
    REQUIRE(delta.z == Approx(0.0).margin(1e-9));
    REQUIRE(delta.y == Approx(5.0).margin(1e-9));
}

TEST_CASE("Camera move_world_up keeps the orbit target distance", "[view_camera]") {
    robcraft::renderer::Camera cam;
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));
    cam.set_orbit_target(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    double d0 = (cam.position() - cam.orbit_target()).length();
    cam.move_world_up(5.0f);
    REQUIRE(cam.orbit_target().y == Approx(5.0).margin(1e-9));
    REQUIRE((cam.position() - cam.orbit_target()).length() == Approx(d0).margin(1e-6));
}
