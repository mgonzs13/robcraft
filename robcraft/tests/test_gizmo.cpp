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

#include "robcraft/editor/gizmo_math.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/renderer/camera.hpp"

using namespace robcraft::engine::math;
using namespace robcraft::renderer;

using Catch::Approx;
using namespace robcraft::editor;
using namespace robcraft::engine;
using namespace robcraft::renderer;

TEST_CASE("project_to_screen centers the camera forward point", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    cam.set_position(Vec3(0, 0, 10));
    cam.look_at(Vec3(0, 0, 0));
    Vec2 c = project_to_screen(cam, 800, 600, Vec3(0, 0, 0));
    REQUIRE(c.x == Approx(400.0).margin(1.0));
    REQUIRE(c.y == Approx(300.0).margin(1.0));
}

TEST_CASE("project_to_screen moves right for +X", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    cam.set_position(Vec3(0, 0, 10));
    cam.look_at(Vec3(0, 0, 0));
    Vec2 c = project_to_screen(cam, 800, 600, Vec3(0, 0, 0));
    Vec2 r = project_to_screen(cam, 800, 600, Vec3(1, 0, 0));
    REQUIRE(r.x > c.x);
}

TEST_CASE("project_to_screen flips Y (down is positive)", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    cam.set_position(Vec3(0, 0, 10));
    cam.look_at(Vec3(0, 0, 0));
    Vec2 c = project_to_screen(cam, 800, 600, Vec3(0, 0, 0));
    Vec2 up = project_to_screen(cam, 800, 600, Vec3(0, 2, 0));
    REQUIRE(up.y < c.y);
}

TEST_CASE("point_segment_distance", "[gizmo]") {
    Vec2 a(0, 0), b(10, 0);
    REQUIRE(point_segment_distance(Vec2(5, 3), a, b) == Approx(3.0));
    REQUIRE(point_segment_distance(Vec2(-5, 0), a, b) == Approx(5.0));
    REQUIRE(point_segment_distance(Vec2(15, 0), a, b) == Approx(5.0));
    REQUIRE(point_segment_distance(Vec2(0, 0), a, b) == Approx(0.0));
    REQUIRE(point_segment_distance(Vec2(3, 4), Vec2(0, 0), Vec2(0, 0)) == Approx(5.0));
}

TEST_CASE("ray_plane_intersect", "[gizmo]") {
    Ray r{Vec3(0, 10, 0), Vec3(0, -1, 0)};
    auto t = ray_plane_intersect(r, Vec3(0, 4, 0), Vec3(0, 1, 0));
    REQUIRE(t.has_value());
    REQUIRE(*t == Approx(6.0));

    Ray parallel{Vec3(0, 10, 0), Vec3(1, 0, 0)};
    REQUIRE(!ray_plane_intersect(parallel, Vec3(0, 4, 0), Vec3(0, 1, 0)).has_value());

    Ray behind{Vec3(0, 0, 0), Vec3(0, -1, 0)};
    REQUIRE(!ray_plane_intersect(behind, Vec3(0, 4, 0), Vec3(0, 1, 0)).has_value());
}

TEST_CASE("yaw_angle cardinal directions", "[gizmo]") {
    Vec3 c(0, 0, 0);
    REQUIRE(yaw_angle(c, Vec3(0, 0, 1)) == Approx(0.0));
    REQUIRE(yaw_angle(c, Vec3(1, 0, 0)) == Approx(robcraft::engine::math::kPi / 2.0));
    REQUIRE(yaw_angle(c, Vec3(-1, 0, 0)) == Approx(-robcraft::engine::math::kPi / 2.0));
    REQUIRE(yaw_angle(c, Vec3(0, 0, -1)) == Approx(robcraft::engine::math::kPi).margin(1e-9));
}

TEST_CASE("snap_angle rounds to the 15 degree grid", "[gizmo]") {
    REQUIRE(snap_angle(0.0) == Approx(0.0));
    REQUIRE(snap_angle(10.0 * robcraft::engine::math::kPi / 180.0) ==
            Approx(15.0 * robcraft::engine::math::kPi / 180.0));
    REQUIRE(snap_angle(-10.0 * robcraft::engine::math::kPi / 180.0) ==
            Approx(-15.0 * robcraft::engine::math::kPi / 180.0));
    REQUIRE(snap_angle(0.5, 0.0) == Approx(0.5));
    REQUIRE(snap_angle(0.5, -15.0) == Approx(0.5));
}

TEST_CASE("gizmo_world_size grows with camera distance", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    cam.set_position(Vec3(0, 0, 10));
    cam.look_at(Vec3(0, 0, 0));
    double d10 = gizmo_world_size(cam, Vec3(0, 0, 0), 600, 90.0);
    REQUIRE(d10 > 0.0);
    cam.set_position(Vec3(0, 0, 20));
    cam.look_at(Vec3(0, 0, 0));
    double d20 = gizmo_world_size(cam, Vec3(0, 0, 0), 600, 90.0);
    REQUIRE(d20 > d10);
}

TEST_CASE("gizmo keeps a constant on-screen size across zoom levels", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    const int vp_w = 800, vp_h = 600;
    for (double dist : {10.0, 20.0, 40.0, 5.0}) {
        cam.set_position(Vec3(0, 0, dist));
        cam.look_at(Vec3(0, 0, 0));
        double s = gizmo_world_size(cam, Vec3(0, 0, 0), vp_h, 140.0);
        Vec2 c = project_to_screen(cam, vp_w, vp_h, Vec3(0, 0, 0));
        Vec2 tip = project_to_screen(cam, vp_w, vp_h, Vec3(0, s, 0));
        REQUIRE(std::abs(tip.y - c.y) == Approx(140.0).margin(0.5));
    }
}

TEST_CASE("pick_yaw_ring hits every point on a tilted ring", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    cam.set_position(Vec3(0, 10, 10));
    cam.look_at(Vec3(0, 0, 0));
    const int vp_w = 800, vp_h = 600;
    const double R = 5.0;
    for (int i = 0; i < 8; ++i) {
        double ang = 2.0 * robcraft::engine::math::kPi * i / 8;
        Vec3 p(R * std::cos(ang), 0.0, R * std::sin(ang));
        Vec2 s = project_to_screen(cam, vp_w, vp_h, p);
        REQUIRE(pick_yaw_ring(cam, s.x, s.y, 0, 0, vp_w, vp_h, Vec3(0, 0, 0), R, 0.72, 12.0));
    }
}

TEST_CASE("pick_yaw_ring hits the ring from the default editor camera", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    cam.set_position(Vec3(50, 40, 50));
    cam.look_at(Vec3(0, 0, 0));
    const int vp_w = 800, vp_h = 600;
    const double R = 5.0;
    for (int i = 0; i < 8; ++i) {
        double ang = 2.0 * robcraft::engine::math::kPi * i / 8;
        Vec3 p(R * std::cos(ang), 0.0, R * std::sin(ang));
        Vec2 s = project_to_screen(cam, vp_w, vp_h, p);
        REQUIRE(pick_yaw_ring(cam, s.x, s.y, 0, 0, vp_w, vp_h, Vec3(0, 0, 0), R, 0.72, 12.0));
    }
}

TEST_CASE("pick_yaw_ring rejects empty space around the ring", "[gizmo]") {
    Camera cam;
    cam.set_perspective(60.0f, 4.0f / 3.0f, 0.1f, 500.0f);
    cam.set_position(Vec3(50, 40, 50));
    cam.look_at(Vec3(0, 0, 0));
    const int vp_w = 800, vp_h = 600;
    const double R = 5.0;
    const Vec3 center(0, 0, 0);
    Vec2 c = project_to_screen(cam, vp_w, vp_h, center);
    Vec2 p_out = project_to_screen(cam, vp_w, vp_h, center + Vec3(R, 0, 0));
    double a_px = (p_out - c).length();
    // The ring's vertical screen extent is smaller than its horizontal extent;
    // a cursor straight above the center at the major-axis distance is empty
    // space, which the old screen-circle test wrongly grabbed.
    REQUIRE(!pick_yaw_ring(cam, c.x, c.y - a_px, 0, 0, vp_w, vp_h, center, R, 0.72, 12.0));
    // Clearly outside the outer radius.
    Vec2 far = project_to_screen(cam, vp_w, vp_h, Vec3(R * 1.6, 0, 0));
    REQUIRE(!pick_yaw_ring(cam, far.x, far.y, 0, 0, vp_w, vp_h, center, R, 0.72, 12.0));
    // Inside the inner hole (the ring center).
    Vec2 hole = project_to_screen(cam, vp_w, vp_h, center);
    REQUIRE(!pick_yaw_ring(cam, hole.x, hole.y, 0, 0, vp_w, vp_h, center, R, 0.72, 12.0));
}

TEST_CASE("shortest_angle_delta crosses the atan2 seam", "[gizmo]") {
    REQUIRE(shortest_angle_delta(0.0, 1.0) == Approx(1.0));
    REQUIRE(shortest_angle_delta(0.0, 0.0) == Approx(0.0));
    // Slightly past +180 degrees is reported by atan2 as ~-180 degrees.
    REQUIRE(shortest_angle_delta(0.0, robcraft::engine::math::kPi + 0.01) ==
            Approx(-(robcraft::engine::math::kPi - 0.01)).margin(1e-9));
    // Continuing past the seam only advances by the short step.
    REQUIRE(shortest_angle_delta(robcraft::engine::math::kPi - 0.01,
                                 -robcraft::engine::math::kPi + 0.01) == Approx(0.02).margin(1e-9));
}
