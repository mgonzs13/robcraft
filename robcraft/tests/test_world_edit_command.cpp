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
#include <memory>

#include "robcraft/editor/command/world_edit_command.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/engine/world/world.hpp"

using namespace robcraft::editor::command;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::world;
using Catch::Approx;

TEST_CASE("WorldEditCommand place entity undo/redo", "[undo]") {
    robcraft::engine::world::World world;

    // World is already in the after-state: entity exists.
    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"wall_9"});
    world.add_component<robcraft::engine::ecs::Transform3D>(
        e, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(4.0, 0.0, 0.0)});
    auto after = robcraft::editor::command::EntitySnapshot::capture(world, e);

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before;
    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after_list;
    after_list.push_back(std::move(after));
    robcraft::editor::command::WorldEditCommand cmd(world, "place wall", std::move(before),
                                                    std::move(after_list), {});

    cmd.undo();
    REQUIRE(!world.valid(e));

    cmd.execute();
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "wall_9");
}

TEST_CASE("WorldEditCommand delete entity undo/redo", "[undo]") {
    robcraft::engine::world::World world;

    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"tree_2"});
    world.add_component<robcraft::engine::ecs::Transform3D>(e,
                                                            robcraft::engine::ecs::Transform3D{});
    auto before = robcraft::editor::command::EntitySnapshot::capture(world, e);

    // World is already in the after-state: entity deleted.
    world.destroy_entity(e);

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before_list;
    before_list.push_back(std::move(before));
    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after_list;
    robcraft::editor::command::WorldEditCommand cmd(world, "delete", std::move(before_list),
                                                    std::move(after_list), {});

    cmd.undo();
    REQUIRE(world.valid(e));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "tree_2");

    cmd.execute();
    REQUIRE(!world.valid(e));
}

TEST_CASE("WorldEditCommand delete restores entity and its non-walkable cell", "[undo]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(8, 8, 1.0));

    // Robot-like entity resting on a cell its placement made non-walkable.
    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(e,
                                                     robcraft::engine::ecs::Name{"robot_mike_1"});
    world.add_component<robcraft::engine::ecs::Transform3D>(
        e, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(0.0, 0.0, 0.0)});

    auto* tf = world.get_component<robcraft::engine::ecs::Transform3D>(e);
    int cx, cz;
    world.terrain().world_to_cell(tf->position.x, tf->position.z, cx, cz);
    REQUIRE(world.terrain().in_bounds(cx, cz));
    world.terrain().set_walkable(cx, cz, false);
    REQUIRE(!world.terrain().is_walkable(cx, cz));

    auto before = robcraft::editor::command::EntitySnapshot::capture(world, e);
    robcraft::editor::command::TerrainCellDelta cell;
    cell.cx = cx;
    cell.cz = cz;
    cell.height_before = cell.height_after = world.terrain().height_at(cx, cz);
    cell.walkable_before = false;
    cell.walkable_after = true;
    cell.cliff_before = cell.cliff_after = world.terrain().cliff_level(cx, cz);
    cell.type_before = cell.type_after = static_cast<uint8_t>(world.terrain().terrain_type(cx, cz));
    cell.water_before =
        world.terrain().has_water(cx, cz) ? 1.0f : robcraft::engine::world::Terrain::WATER_OFF;
    cell.water_after = cell.water_before;

    // Simulate the after-state: entity deleted, cell walkability restored.
    world.destroy_entity(e);
    world.terrain().set_walkable(cx, cz, true);

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before_list;
    before_list.push_back(std::move(before));
    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after_list;
    robcraft::editor::command::WorldEditCommand cmd(world, "delete", std::move(before_list),
                                                    std::move(after_list), {cell});

    cmd.undo();
    REQUIRE(world.valid(e));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "robot_mike_1");
    REQUIRE(!world.terrain().is_walkable(cx, cz));

    cmd.execute();
    REQUIRE(!world.valid(e));
    REQUIRE(world.terrain().is_walkable(cx, cz));

    // Cycle again to confirm idempotence.
    cmd.undo();
    REQUIRE(world.valid(e));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "robot_mike_1");
    REQUIRE(!world.terrain().is_walkable(cx, cz));

    cmd.execute();
    REQUIRE(!world.valid(e));
    REQUIRE(world.terrain().is_walkable(cx, cz));
}

TEST_CASE("WorldEditCommand transform edit undo/redo", "[undo]") {
    robcraft::engine::world::World world;

    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(
        e, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(1.0, 0.0, 0.0)});

    auto before = robcraft::editor::command::EntitySnapshot::capture(world, e);
    world.get_component<robcraft::engine::ecs::Transform3D>(e)->position =
        robcraft::engine::math::Vec3(5.0, 0.0, 0.0);
    auto after = robcraft::editor::command::EntitySnapshot::capture(world, e);

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before_list;
    before_list.push_back(std::move(before));
    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after_list;
    after_list.push_back(std::move(after));
    robcraft::editor::command::WorldEditCommand cmd(world, "move", std::move(before_list),
                                                    std::move(after_list), {});

    cmd.undo();
    REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(e)->position.x == Approx(1.0));

    cmd.execute();
    REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(e)->position.x == Approx(5.0));
}

TEST_CASE("WorldEditCommand terrain cell edit undo/redo", "[undo]") {
    robcraft::engine::world::World world;
    world.set_terrain(robcraft::engine::world::Terrain(8, 8, 1.0));

    // World is in the after-state: height 3.0, non-walkable.
    world.terrain().set_height(2, 3, 3.0f);
    world.terrain().set_walkable(2, 3, false);
    world.terrain().set_water(2, 3, true);

    robcraft::editor::command::TerrainCellDelta cell;
    cell.cx = 2;
    cell.cz = 3;
    cell.height_before = 0.0f;
    cell.height_after = 3.0f;
    cell.walkable_before = true;
    cell.walkable_after = false;
    cell.cliff_before = 0;
    cell.cliff_after = 0;
    cell.type_before = 0;
    cell.type_after = 1;
    cell.water_before = robcraft::engine::world::Terrain::WATER_OFF;
    cell.water_after = 1.0f;

    robcraft::editor::command::WorldEditCommand cmd(world, "paint", {}, {}, {cell});

    cmd.undo();
    REQUIRE(world.terrain().height_at(2, 3) == Approx(0.0f));
    REQUIRE(world.terrain().is_walkable(2, 3));
    REQUIRE(!world.terrain().has_water(2, 3));

    cmd.execute();
    REQUIRE(world.terrain().height_at(2, 3) == Approx(3.0f));
    REQUIRE(!world.terrain().is_walkable(2, 3));
    REQUIRE(world.terrain().has_water(2, 3));
}

TEST_CASE("WorldEditCommand undo is idempotent across repeated cycles", "[undo]") {
    robcraft::engine::world::World world;

    robcraft::engine::core::Entity e = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{"pillar_2"});
    auto after = robcraft::editor::command::EntitySnapshot::capture(world, e);

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before;
    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after_list;
    after_list.push_back(std::move(after));
    robcraft::editor::command::WorldEditCommand cmd(world, "place", std::move(before),
                                                    std::move(after_list), {});

    for (int i = 0; i < 3; ++i) {
        cmd.undo();
        REQUIRE(!world.valid(e));
        cmd.execute();
        REQUIRE(world.get_component<robcraft::engine::ecs::Name>(e)->value == "pillar_2");
    }
}

TEST_CASE("WorldEditCommand composite edit destroys and creates entities", "[undo]") {
    robcraft::engine::world::World world;

    // Keeper is never referenced by the command and must survive untouched.
    robcraft::engine::core::Entity keeper = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(keeper, robcraft::engine::ecs::Name{"keeper"});

    // Victim exists only in the before-state: the edit deletes it.
    robcraft::engine::core::Entity victim = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(victim, robcraft::engine::ecs::Name{"victim"});

    // Mover is edited: position changes between before and after.
    robcraft::engine::core::Entity mover = world.create_entity();
    world.add_component<robcraft::engine::ecs::Transform3D>(
        mover, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(1.0, 0.0, 0.0)});

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before;
    before.push_back(robcraft::editor::command::EntitySnapshot::capture(world, victim));
    before.push_back(robcraft::editor::command::EntitySnapshot::capture(world, mover));

    // Created exists only in the after-state: the edit spawns it.
    // Allocate it before destroying the victim so it gets a fresh id
    // (freed ids are reused FIFO, which would otherwise alias victim).
    robcraft::engine::core::Entity created = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(created,
                                                     robcraft::engine::ecs::Name{"created"});

    // Simulate the after-state: victim deleted, mover moved.
    world.destroy_entity(victim);
    world.get_component<robcraft::engine::ecs::Transform3D>(mover)->position =
        robcraft::engine::math::Vec3(5.0, 0.0, 0.0);

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after_list;
    after_list.push_back(robcraft::editor::command::EntitySnapshot::capture(world, mover));
    after_list.push_back(robcraft::editor::command::EntitySnapshot::capture(world, created));

    robcraft::editor::command::WorldEditCommand cmd(world, "merge", std::move(before),
                                                    std::move(after_list), {});

    cmd.undo();
    REQUIRE(world.valid(victim));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(victim)->value == "victim");
    REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(mover)->position.x ==
            Approx(1.0));
    REQUIRE(!world.valid(created));
    REQUIRE(world.valid(keeper));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(keeper)->value == "keeper");

    cmd.execute();
    REQUIRE(!world.valid(victim));
    REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(mover)->position.x ==
            Approx(5.0));
    REQUIRE(world.valid(created));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(created)->value == "created");
    REQUIRE(world.valid(keeper));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(keeper)->value == "keeper");
}

TEST_CASE("WorldEditCommand merge with id aliasing reuses freed id", "[undo]") {
    robcraft::engine::world::World world;

    // Before-only: an absorbed wall (id A). merge_target is edited in place.
    robcraft::engine::core::Entity absorbed = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(absorbed,
                                                     robcraft::engine::ecs::Name{"wall_absorbed"});

    robcraft::engine::core::Entity merge_target = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(merge_target,
                                                     robcraft::engine::ecs::Name{"wall_merge"});
    world.add_component<robcraft::engine::ecs::Transform3D>(
        merge_target,
        robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(0.0, 0.0, 0.0)});

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before;
    before.push_back(robcraft::editor::command::EntitySnapshot::capture(world, merge_target));
    before.push_back(robcraft::editor::command::EntitySnapshot::capture(world, absorbed));

    // Simulate the after-state merge: move the target, destroy the absorbed
    // wall, then create a corner pillar that reuses the freed id (FIFO list).
    world.get_component<robcraft::engine::ecs::Transform3D>(merge_target)->position =
        robcraft::engine::math::Vec3(10.0, 0.0, 10.0);
    world.destroy_entity(absorbed);
    robcraft::engine::core::Entity pillar = world.create_entity();
    world.add_component<robcraft::engine::ecs::Name>(pillar,
                                                     robcraft::engine::ecs::Name{"wall_pillar"});
    world.add_component<robcraft::engine::ecs::Transform3D>(
        pillar, robcraft::engine::ecs::Transform3D{robcraft::engine::math::Vec3(5.0, 0.75, 5.0)});
    REQUIRE(pillar == absorbed);  // the aliasing case genuinely occurs

    std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after;
    after.push_back(robcraft::editor::command::EntitySnapshot::capture(world, merge_target));
    after.push_back(robcraft::editor::command::EntitySnapshot::capture(world, pillar));

    robcraft::editor::command::WorldEditCommand cmd(world, "merge", std::move(before),
                                                    std::move(after), {});

    for (int i = 0; i < 2; ++i) {
        cmd.undo();
        // The absorbed wall's identity is restored onto the shared id; the
        // pillar's state is gone from it.
        REQUIRE(world.valid(absorbed));
        REQUIRE(world.get_component<robcraft::engine::ecs::Name>(absorbed)->value ==
                "wall_absorbed");
        REQUIRE(world.get_component<robcraft::engine::ecs::Name>(pillar)->value == "wall_absorbed");
        REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(merge_target)->position.x ==
                Approx(0.0));
        REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(merge_target)->position.z ==
                Approx(0.0));

        cmd.execute();
        // The pillar's identity is restored onto the shared id; the absorbed
        // wall's state is gone from it.
        REQUIRE(world.valid(pillar));
        REQUIRE(world.get_component<robcraft::engine::ecs::Name>(pillar)->value == "wall_pillar");
        REQUIRE(world.get_component<robcraft::engine::ecs::Name>(absorbed)->value == "wall_pillar");
        REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(merge_target)->position.x ==
                Approx(10.0));
        REQUIRE(world.get_component<robcraft::engine::ecs::Transform3D>(merge_target)->position.z ==
                Approx(10.0));
    }
}

TEST_CASE("WorldEditCommand cross-command redo preserves entity ids", "[undo]") {
    robcraft::engine::world::World world;
    robcraft::editor::command::UndoStack stack;

    // Two separate place commands, one entity each.
    auto place = [&](const char* nm) {
        robcraft::engine::core::Entity e = world.create_entity();
        world.add_component<robcraft::engine::ecs::Name>(e, robcraft::engine::ecs::Name{nm});
        std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> before;
        std::vector<std::unique_ptr<robcraft::editor::command::EntitySnapshot>> after;
        after.push_back(robcraft::editor::command::EntitySnapshot::capture(world, e));
        stack.execute(std::make_unique<robcraft::editor::command::WorldEditCommand>(
            world, "place", std::move(before), std::move(after),
            std::vector<robcraft::editor::command::TerrainCellDelta>{}));
        return e;
    };

    robcraft::engine::core::Entity x = place("wall_X");
    robcraft::engine::core::Entity y = place("tree_Y");

    stack.undo();  // removes y
    stack.undo();  // removes x
    REQUIRE(!world.valid(x));
    REQUIRE(!world.valid(y));

    stack.redo();  // restores x
    stack.redo();  // restores y
    // Both entities must survive at their recorded ids — a regression guard
    // against free-list id theft across commands.
    REQUIRE(world.valid(x));
    REQUIRE(world.valid(y));
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(x)->value == "wall_X");
    REQUIRE(world.get_component<robcraft::engine::ecs::Name>(y)->value == "tree_Y");
}
