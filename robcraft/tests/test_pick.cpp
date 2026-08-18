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

#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/camera.hpp"
#include "robcraft/renderer/pick.hpp"

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;

using Catch::Approx;

TEST_CASE("Pick returns terrain point under cursor", "[pick]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    auto p = robcraft::renderer::pick_world_point(cam, world, 400.0, 300.0, 0, 0, 800, 600);
    REQUIRE(p.has_value());
    REQUIRE(p->x == Approx(0.0).margin(0.5));
    REQUIRE(p->z == Approx(0.0).margin(0.5));
    REQUIRE(p->y == Approx(0.0).margin(0.5));
}

TEST_CASE("Pick prefers objects over terrain", "[pick]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    auto e = world.create_entity();
    robcraft::engine::ecs::Transform3D tf;
    tf.position = robcraft::engine::math::Vec3(0.0, 3.0, 5.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(e, tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        e, robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(1.0, 1.0, 1.0)});

    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    auto p = robcraft::renderer::pick_world_point(cam, world, 400.0, 300.0, 0, 0, 800, 600);
    REQUIRE(p.has_value());
    double dist_obj = (cam.position() - *p).length();
    double dist_terrain = std::sqrt(500.0);  // camera -> origin
    REQUIRE(dist_obj < dist_terrain - 1.0);
}

TEST_CASE("Pick returns nullopt with no terrain or objects", "[pick]") {
    robcraft::engine::world::World world;

    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    auto p = robcraft::renderer::pick_world_point(cam, world, 400.0, 300.0, 0, 0, 800, 600);
    REQUIRE(!p.has_value());
}

TEST_CASE("Pick returns nullopt when ray misses terrain", "[pick]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 5.0, 0.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 50.0, 0.0));  // looking straight up

    auto p = robcraft::renderer::pick_world_point(cam, world, 400.0, 300.0, 0, 0, 800, 600);
    REQUIRE(!p.has_value());
}

TEST_CASE("Pick ignores objects off the cursor ray", "[pick]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    auto e = world.create_entity();
    robcraft::engine::ecs::Transform3D tf;
    tf.position = robcraft::engine::math::Vec3(5.0, 1.0, 0.0);
    world.add_component<robcraft::engine::ecs::Transform3D>(e, tf);
    world.add_component<robcraft::engine::collision::BoxCollider>(
        e, robcraft::engine::collision::BoxCollider{robcraft::engine::math::Vec3(1.0, 1.0, 1.0)});

    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    auto p = robcraft::renderer::pick_world_point(cam, world, 400.0, 300.0, 0, 0, 800, 600);
    REQUIRE(p.has_value());
    REQUIRE(p->x == Approx(0.0).margin(0.5));
    REQUIRE(p->z == Approx(0.0).margin(0.5));
    REQUIRE(p->y == Approx(0.0).margin(0.5));
}

TEST_CASE("Pick builds perpendicular basis when looking straight down", "[pick]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(32, 32, 2.0));

    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 0.0));
    cam.look_at(robcraft::engine::math::Vec3(
        0.0, 0.0, 0.0));  // forward parallel to world up — degenerate basis

    auto p = robcraft::renderer::pick_world_point(cam, world, 100.0, 300.0, 0, 0, 800, 600);
    REQUIRE(p.has_value());
    REQUIRE(p->x < -1.0);  // cursor left of center must pick terrain left of origin
    REQUIRE(p->y == Approx(0.0).margin(0.5));
    REQUIRE(p->z == Approx(0.0).margin(0.5));
}

TEST_CASE("Pick returns point on horizontal plane", "[pick]") {
    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 0.0, 0.0));

    auto p = robcraft::renderer::pick_point_on_plane(cam, 400.0, 300.0, 0, 0, 800, 600, 0.0);
    REQUIRE(p.has_value());
    REQUIRE(p->y == Approx(0.0).margin(0.01));
    REQUIRE(p->x == Approx(0.0).margin(0.5));
    REQUIRE(p->z == Approx(0.0).margin(0.5));
}

TEST_CASE("Pick returns nullopt when ray parallel to plane", "[pick]") {
    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 20.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 10.0, 0.0));  // forward has y == 0

    auto p = robcraft::renderer::pick_point_on_plane(cam, 400.0, 300.0, 0, 0, 800, 600, 0.0);
    REQUIRE(!p.has_value());
}

TEST_CASE("Pick returns nullopt when ray points away from plane", "[pick]") {
    robcraft::renderer::Camera cam;
    cam.set_perspective(60.0f, 800.0f / 600.0f, 0.1f, 500.0f);
    cam.set_position(robcraft::engine::math::Vec3(0.0, 10.0, 0.0));
    cam.look_at(robcraft::engine::math::Vec3(0.0, 50.0, 0.0));  // looking up; plane y=0 is behind

    auto p = robcraft::renderer::pick_point_on_plane(cam, 400.0, 300.0, 0, 0, 800, 600, 0.0);
    REQUIRE(!p.has_value());
}
