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

#include "robcraft/editor/editor_place.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/collision/raycast.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/math/cell_range.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/renderer/model.hpp"
#include "robcraft/renderer/model_paths.hpp"
#include "robcraft/renderer/pick.hpp"
#include "robcraft/renderer/robot_factory.hpp"

namespace robcraft::editor {

using namespace robcraft::engine::collision;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;
using namespace robcraft::renderer;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::robots::differential_drive;

EditorPlacement::EditorPlacement(EditorApp& app) : app_(app) {}

bool EditorPlacement::cursor_cell(int& cx, int& cz) const {
    if (!this->app_.world_.has_terrain()) return false;
    Ray ray = this->cursor_ray();
    auto hit = this->app_.world_.terrain().raycast(ray);
    if (!hit.has_value()) return false;
    double t = *hit;
    this->app_.world_.terrain().world_to_cell(ray.origin.x + ray.direction.x * t,
                                              ray.origin.z + ray.direction.z * t, cx, cz);
    return this->app_.world_.terrain().in_bounds(cx, cz);
}

Ray EditorPlacement::cursor_ray() const {
    double mx, my;
    glfwGetCursorPos(this->app_.window_, &mx, &my);
    return robcraft::editor::cursor_ray(this->app_.editor_camera_, mx, my, this->app_.viewport_x_,
                                        this->app_.viewport_y_, this->app_.viewport_w_,
                                        this->app_.viewport_h_);
}

Entity EditorPlacement::pick_entity() const {
    Ray ray = this->cursor_ray();
    double closest = 1e9;
    Entity hit = INVALID_ENTITY;
    auto* cs = this->app_.world_.store<BoxCollider>();
    auto* ts = this->app_.world_.store<Transform3D>();
    if (cs && ts) {
        for (auto& [e, col] : *cs) {
            auto* tf = ts->get(e);
            if (!tf) continue;
            auto aabb = AABB::from_box(tf->position, col, tf->rotation);
            auto h = ray_aabb_intersection(ray, aabb);
            if (h.has_value() && *h < closest) {
                closest = *h;
                hit = e;
            }
        }
    }
    if (ts) {
        for (auto& [e, tf] : *ts) {
            if (cs && cs->has(e)) continue;
            Vec3 half = tf.scale * 0.5;
            AABB box{tf.position - half, tf.position + half};
            auto h = ray_aabb_intersection(ray, box);
            if (h.has_value() && *h < closest) {
                closest = *h;
                hit = e;
            }
        }
    }
    return hit;
}

void EditorPlacement::handle_select_click() {
    Entity hit = this->pick_entity();
    bool ctrl = ImGui::GetIO().KeyCtrl;
    if (hit != INVALID_ENTITY && ctrl) {
        this->toggle_selection(hit);
    } else if (hit != INVALID_ENTITY) {
        this->clear_selection();
        this->add_selection(hit);
    } else if (!ctrl) {
        this->clear_selection();
    }
}

void EditorPlacement::handle_place_press() {
    int cx, cz;
    if (!this->cursor_cell(cx, cz)) return;
    if (this->app_.placeable_ == PlaceableType::Wall ||
        this->app_.placeable_ == PlaceableType::Floor) {
        this->app_.drag_active_ = true;
        this->app_.drag_anchor_x_ = cx;
        this->app_.drag_anchor_z_ = cz;
        this->app_.drag_cur_x_ = cx;
        this->app_.drag_cur_z_ = cz;
        return;
    }
    this->place_single(cx, cz);
}

void EditorPlacement::handle_place_release() {
    if (!this->app_.drag_active_) return;
    this->app_.drag_active_ = false;
    int cx, cz;
    if (!this->cursor_cell(cx, cz)) return;
    this->app_.drag_cur_x_ = cx;
    this->app_.drag_cur_z_ = cz;
    if (this->app_.placeable_ == PlaceableType::Wall)
        this->place_wall_run(this->app_.drag_anchor_x_, this->app_.drag_anchor_z_, cx, cz);
    else
        this->place_floor_rect(this->app_.drag_anchor_x_, this->app_.drag_anchor_z_, cx, cz);
}

void EditorPlacement::place_wall_run(int ax, int az, int bx, int bz) {
    if (!this->app_.world_.has_terrain()) return;
    auto& t = this->app_.world_.terrain();
    CellRange run = wall_run_cells(ax, az, bx, bz);
    std::vector<CellRange> joins = this->find_corner_joins(run);

    // Merge candidates: parallel wall runs overlapping or abutting the union
    Entity merge_target = INVALID_ENTITY;
    std::vector<Entity> absorbed;
    CellRange union_range = run;
    for (int pass = 0; pass < 32; ++pass) {
        Entity next = INVALID_ENTITY;
        CellRange next_union = union_range;
        auto* cs = this->app_.world_.store<BoxCollider>();
        auto* ts = this->app_.world_.store<Transform3D>();
        if (!cs || !ts) break;
        for (auto& [e, col] : *cs) {
            if (!ts->has(e)) continue;
            if (e == merge_target) continue;
            bool already = false;
            for (Entity a : absorbed) {
                if (a == e) already = true;
            }
            if (already) continue;
            CellRange u;
            if (this->wall_mergeable(union_range, e, u)) {
                next = e;
                next_union = u;
                break;
            }
        }
        if (next == INVALID_ENTITY) break;
        if (merge_target == INVALID_ENTITY)
            merge_target = next;
        else
            absorbed.push_back(next);
        union_range = next_union;
    }

    // Gate: allow non-walkable cells that are joined endpoints, pillar-filled
    // endpoints, or covered by a merging wall.
    auto covered_by_merges = [&](int x, int z) {
        Vec3 c = t.cell_center_world(x, z);
        auto* ts = this->app_.world_.store<Transform3D>();
        auto* cs = this->app_.world_.store<BoxCollider>();
        if (!ts || !cs) return false;
        std::vector<Entity> walls;
        if (merge_target != INVALID_ENTITY) walls.push_back(merge_target);
        for (Entity w : absorbed) walls.push_back(w);
        for (Entity w : walls) {
            auto* tf = ts->get(w);
            auto* col = cs->get(w);
            if (!tf || !col) continue;
            Vec3 half = col->half_extents;
            if (c.x >= tf->position.x - half.x && c.x <= tf->position.x + half.x &&
                c.z >= tf->position.z - half.z && c.z <= tf->position.z + half.z)
                return true;
        }
        return false;
    };
    for (int z = run.z0; z <= run.z1; ++z) {
        for (int x = run.x0; x <= run.x1; ++x) {
            if (!t.is_walkable(x, z)) {
                bool joined = false;
                for (const CellRange& j : joins) {
                    if (j.x0 == x && j.z0 == z) joined = true;
                }
                if (!joined && !this->cell_has_pillar(x, z) && !covered_by_merges(x, z)) return;
            }
        }
    }

    // Guard before recording: the merge target must still carry the components
    // the merge rewrites, so bail before the recorder is dirtied.
    if (merge_target != INVALID_ENTITY &&
        (!this->app_.world_.get_component<Transform3D>(merge_target) ||
         !this->app_.world_.get_component<BoxCollider>(merge_target))) {
        return;
    }

    this->app_.undo_recorder_.begin();
    if (merge_target != INVALID_ENTITY)
        this->app_.undo_recorder_.record_before_entity(merge_target);
    for (Entity w : absorbed) this->app_.undo_recorder_.record_before_entity(w);
    for (int z = union_range.z0; z <= union_range.z1; ++z)
        for (int x = union_range.x0; x <= union_range.x1; ++x)
            this->app_.undo_recorder_.record_before_cell(x, z);

    if (merge_target != INVALID_ENTITY) {
        // Extend the existing wall to cover the union
        Transform3D tf;
        this->fill_wall_run_transform(t, union_range, tf);
        auto* mtf = this->app_.world_.get_component<Transform3D>(merge_target);
        auto* mcol = this->app_.world_.get_component<BoxCollider>(merge_target);
        mtf->position = tf.position;
        mtf->scale = tf.scale;
        mcol->half_extents = tf.scale * 0.5;
        for (Entity w : absorbed) {
            this->restore_walkable_under(w);
            this->app_.world_.destroy_entity(w);
        }
        for (int z = union_range.z0; z <= union_range.z1; ++z) {
            for (int x = union_range.x0; x <= union_range.x1; ++x) {
                t.set_walkable(x, z, false);
            }
        }
        std::vector<Entity> created;
        for (const CellRange& j : this->find_corner_joins(union_range)) {
            auto cc = t.cell_center_world(j.x0, j.z0);
            float ground = t.height_at_world(cc.x, cc.z);
            Entity p = this->app_.world_.create_entity();
            Transform3D ptf;
            ptf.position = Vec3(cc.x, ground + 1.5, cc.z);
            ptf.scale = Vec3(t.cell_size(), 3.0, t.cell_size());
            this->app_.world_.add_component<Transform3D>(p, ptf);
            this->app_.world_.add_component<Name>(p, Name{"wall_" + std::to_string(p)});
            this->app_.world_.add_component<BoxCollider>(p, BoxCollider{ptf.scale * 0.5});
            created.push_back(p);
        }
        this->app_.tools_.mark_modified();
        this->app_.undo_recorder_.record_after_entity(merge_target);
        for (Entity p : created) this->app_.undo_recorder_.record_after_entity(p);
        this->app_.commit_undo("Place wall");
        this->clear_selection();
        this->add_selection(merge_target);
        return;
    }

    // No merge: create a new run entity (existing behavior)
    Entity e = this->app_.world_.create_entity();
    Transform3D tf;
    this->fill_wall_run_transform(t, run, tf);
    this->app_.world_.add_component<Transform3D>(e, tf);
    this->app_.world_.add_component<Name>(e, Name{"wall_" + std::to_string(e)});
    this->app_.world_.add_component<BoxCollider>(e, BoxCollider{tf.scale * 0.5});
    std::vector<Entity> created;
    for (const CellRange& j : joins) {
        auto cc = t.cell_center_world(j.x0, j.z0);
        float ground = t.height_at_world(cc.x, cc.z);
        Entity p = this->app_.world_.create_entity();
        Transform3D ptf;
        ptf.position = Vec3(cc.x, ground + 1.5, cc.z);
        ptf.scale = Vec3(t.cell_size(), 3.0, t.cell_size());
        this->app_.world_.add_component<Transform3D>(p, ptf);
        this->app_.world_.add_component<Name>(p, Name{"wall_" + std::to_string(p)});
        this->app_.world_.add_component<BoxCollider>(p, BoxCollider{ptf.scale * 0.5});
        created.push_back(p);
    }
    for (int z = run.z0; z <= run.z1; ++z) {
        for (int x = run.x0; x <= run.x1; ++x) {
            t.set_walkable(x, z, false);
        }
    }
    this->app_.tools_.mark_modified();
    this->app_.undo_recorder_.record_after_entity(e);
    for (Entity p : created) this->app_.undo_recorder_.record_after_entity(p);
    this->app_.commit_undo("Place wall");
    this->clear_selection();
    this->add_selection(e);
}

std::vector<CellRange> EditorPlacement::find_corner_joins(const CellRange& run) const {
    std::vector<CellRange> joins;
    if (!this->app_.world_.has_terrain()) return joins;
    auto* cs = this->app_.world_.store<BoxCollider>();
    auto* ts = this->app_.world_.store<Transform3D>();
    if (!cs || !ts) return joins;
    bool horizontal = cell_range_is_horizontal(run);
    CellRange a{run.x0, run.z0, run.x0, run.z0};
    CellRange b{run.x1, run.z1, run.x1, run.z1};
    std::vector<CellRange> endpoints;
    endpoints.push_back(a);
    if (a.x0 != b.x0 || a.z0 != b.z0) endpoints.push_back(b);
    auto covers_cell = [](const Vec3& half, const Vec3& pos, const Vec3& center) {
        return center.x >= pos.x - half.x && center.x <= pos.x + half.x &&
               center.z >= pos.z - half.z && center.z <= pos.z + half.z;
    };
    for (const CellRange& ep : endpoints) {
        if (this->cell_has_pillar(ep.x0, ep.z0)) continue;  // corner already filled
        Vec3 center = this->app_.world_.terrain().cell_center_world(ep.x0, ep.z0);
        for (auto& [e, col] : *cs) {
            auto* tf = ts->get(e);
            if (!tf) continue;
            auto* nm = this->app_.world_.get_component<Name>(e);
            if (!nm || nm->value.rfind("wall", 0) != 0) continue;
            Vec3 half = col.half_extents;
            if (!covers_cell(half, tf->position, center)) continue;
            bool w_thin_x = half.x <= 0.26;
            if (w_thin_x != horizontal) continue;  // parallel, not perpendicular
            joins.push_back(ep);
            break;
        }
    }
    return joins;
}

bool EditorPlacement::cell_has_pillar(int cx, int cz) const {
    if (!this->app_.world_.has_terrain()) return false;
    auto* cs = this->app_.world_.store<BoxCollider>();
    auto* ts = this->app_.world_.store<Transform3D>();
    if (!cs || !ts) return false;
    Vec3 center = this->app_.world_.terrain().cell_center_world(cx, cz);
    for (auto& [e, col] : *cs) {
        auto* tf = ts->get(e);
        if (!tf) continue;
        auto* nm = this->app_.world_.get_component<Name>(e);
        if (!nm || nm->value.rfind("wall", 0) != 0) continue;
        Vec3 half = col.half_extents;
        if (half.x <= 0.26 || std::abs(half.x - half.z) > 1e-6) continue;  // not a pillar
        bool covers = center.x >= tf->position.x - half.x && center.x <= tf->position.x + half.x &&
                      center.z >= tf->position.z - half.z && center.z <= tf->position.z + half.z;
        if (covers) return true;
    }
    return false;
}

bool EditorPlacement::wall_mergeable(const CellRange& run, Entity e, CellRange& out_union) const {
    auto* tf = this->app_.world_.get_component<Transform3D>(e);
    auto* col = this->app_.world_.get_component<BoxCollider>(e);
    auto* nm = this->app_.world_.get_component<Name>(e);
    if (!tf || !col || !nm || nm->value.rfind("wall", 0) != 0) return false;
    Vec3 half = col->half_extents;
    if (half.x > 0.26 && std::abs(half.x - half.z) < 1e-6) return false;  // corner pillar
    int x0, z0, x1, z1;
    this->app_.world_.terrain().world_to_cell(tf->position.x - half.x, tf->position.z - half.z, x0,
                                              z0);
    this->app_.world_.terrain().world_to_cell(tf->position.x + half.x - 1e-6,
                                              tf->position.z + half.z - 1e-6, x1, z1);
    CellRange wall{x0, z0, x1, z1};
    if (!cell_ranges_merge(run, wall)) return false;
    out_union = {std::min(run.x0, wall.x0), std::min(run.z0, wall.z0), std::max(run.x1, wall.x1),
                 std::max(run.z1, wall.z1)};
    return true;
}

void EditorPlacement::place_floor_rect(int ax, int az, int bx, int bz) {
    if (!this->app_.world_.has_terrain()) return;
    CellRange rect = floor_rect_cells(ax, az, bx, bz);
    this->app_.undo_recorder_.begin();
    Entity e = this->app_.world_.create_entity();
    Transform3D tf;
    this->fill_floor_rect_transform(this->app_.world_.terrain(), rect, tf);
    this->app_.world_.add_component<Transform3D>(e, tf);
    this->app_.world_.add_component<Name>(e, Name{"floor_" + std::to_string(e)});
    this->app_.tools_.mark_modified();
    this->app_.undo_recorder_.record_after_entity(e);
    this->app_.commit_undo("Place floor");
    this->clear_selection();
    this->add_selection(e);
}

void EditorPlacement::fill_wall_run_transform(const Terrain& t, const CellRange& run,
                                              Transform3D& tf) const {
    double cs = t.cell_size();
    bool horizontal = cell_range_is_horizontal(run);
    double wx = (run.x0 + run.x1 + 1) * 0.5 * cs - t.width() * cs * 0.5;
    double wz = (run.z0 + run.z1 + 1) * 0.5 * cs - t.height() * cs * 0.5;
    float ground = t.height_at_world(wx, wz);
    tf.position = Vec3(wx, ground + 1.5, wz);
    if (horizontal) {
        tf.scale = Vec3((run.x1 - run.x0 + 1) * cs, 3.0, 0.5);
    } else {
        tf.scale = Vec3(0.5, 3.0, (run.z1 - run.z0 + 1) * cs);
    }
}

void EditorPlacement::fill_floor_rect_transform(const Terrain& t, const CellRange& rect,
                                                Transform3D& tf) const {
    double cs = t.cell_size();
    double wx = (rect.x0 + rect.x1 + 1) * 0.5 * cs - t.width() * cs * 0.5;
    double wz = (rect.z0 + rect.z1 + 1) * 0.5 * cs - t.height() * cs * 0.5;
    float ground = t.height_at_world(wx, wz);
    tf.position = Vec3(wx, ground + 0.005, wz);
    tf.scale = Vec3((rect.x1 - rect.x0 + 1) * cs, 0.01, (rect.z1 - rect.z0 + 1) * cs);
}

void EditorPlacement::place_single(int cx, int cz) {
    if (!this->app_.world_.has_terrain()) return;
    auto& t = this->app_.world_.terrain();

    // Robots go through the sensor-equipped placement path.
    if (this->app_.placeable_ == PlaceableType::RobotGeorge ||
        this->app_.placeable_ == PlaceableType::RobotLeela ||
        this->app_.placeable_ == PlaceableType::RobotMike ||
        this->app_.placeable_ == PlaceableType::RobotStan) {
        if (!t.is_walkable(cx, cz)) return;
        auto cc = t.cell_center_world(cx, cz);
        this->handle_robot_placement(cc.x, cc.z, this->prefix_for_type(this->app_.placeable_));
        return;
    }

    if (this->app_.placeable_ == PlaceableType::PointLight) {
        if (!t.is_walkable(cx, cz)) return;
        auto cc = t.cell_center_world(cx, cz);
        float ground = t.height_at_world(cc.x, cc.z);
        this->app_.undo_recorder_.begin();
        Entity e = this->app_.world_.create_entity();
        Transform3D tf;
        tf.scale = Vec3(1.0, 1.0, 1.0);
        tf.position = Vec3(cc.x, ground + 0.8, cc.z);
        this->app_.world_.add_component<Transform3D>(e, tf);
        this->app_.world_.add_component<Name>(e, Name{"light_" + std::to_string(e)});
        this->app_.world_.add_component<PointLight>(e, PointLight{});
        this->app_.tools_.mark_modified();
        this->app_.undo_recorder_.record_after_entity(e);
        this->app_.commit_undo("Place light");
        this->clear_selection();
        this->add_selection(e);
        return;
    }

    const char* prefix = this->prefix_for_type(this->app_.placeable_);
    const robcraft::renderer::PlacementSpec* spec =
        robcraft::renderer::placement_spec_for_prefix(prefix);

    // Multi-tile objects (e.g. the space base): occupy a whole footprint.
    if (spec && spec->multi_tile) {
        auto fp = robcraft::renderer::placement_footprint_cells(spec, t.cell_size());
        int x0 = cx - fp.first / 2;
        int z0 = cz - fp.second / 2;
        for (int z = z0; z < z0 + fp.second; ++z) {
            for (int x = x0; x < x0 + fp.first; ++x) {
                if (!t.in_bounds(x, z) || !t.is_walkable(x, z)) return;
            }
        }
        this->app_.undo_recorder_.begin();
        for (int z = z0; z < z0 + fp.second; ++z)
            for (int x = x0; x < x0 + fp.first; ++x)
                this->app_.undo_recorder_.record_before_cell(x, z);
        double wx = (x0 + fp.first * 0.5) * t.cell_size() - t.width() * t.cell_size() * 0.5;
        double wz = (z0 + fp.second * 0.5) * t.cell_size() - t.height() * t.cell_size() * 0.5;
        float ground = t.height_at_world(wx, wz);
        auto model = this->app_.model_cache_.get(spec->model_path);
        Entity e = this->app_.world_.create_entity();
        Transform3D tf;
        tf.scale = spec->base_scale;
        tf.position = Vec3(
            wx, ground + robcraft::renderer::placement_ground_offset(spec, model.get(), tf.scale),
            wz);
        this->app_.world_.add_component<Transform3D>(e, tf);
        this->app_.world_.add_component<Name>(e,
                                              Name{std::string(prefix) + "_" + std::to_string(e)});
        this->app_.world_.add_component<BoxCollider>(
            e, BoxCollider{collider_half_extents(model.get(), tf.scale)});
        for (int z = z0; z < z0 + fp.second; ++z)
            for (int x = x0; x < x0 + fp.first; ++x) t.set_walkable(x, z, false);
        this->app_.tools_.mark_modified();
        this->app_.undo_recorder_.record_after_entity(e);
        this->app_.commit_undo("Place");
        this->clear_selection();
        this->add_selection(e);
        return;
    }

    if (!t.is_walkable(cx, cz)) return;
    this->app_.undo_recorder_.begin();
    this->app_.undo_recorder_.record_before_cell(cx, cz);
    auto cc = t.cell_center_world(cx, cz);

    float ground = t.height_at_world(cc.x, cc.z);
    Entity e = this->app_.world_.create_entity();
    Transform3D tf;
    tf.position = Vec3(cc.x, ground, cc.z);

    double angle = 0.0;
    double s = 1.0;
    bool jitter = !(this->app_.placeable_ == PlaceableType::Bed ||
                    this->app_.placeable_ == PlaceableType::Chair ||
                    this->app_.placeable_ == PlaceableType::Couch ||
                    this->app_.placeable_ == PlaceableType::LightFloor ||
                    this->app_.placeable_ == PlaceableType::Shelf ||
                    this->app_.placeable_ == PlaceableType::Table);
    if (jitter) {
        angle = this->app_.rng_.uniform(-0.26, 0.26);
        s = this->app_.rng_.uniform(0.8, 1.2);
    }
    std::shared_ptr<robcraft::renderer::Model> model;
    if (spec) {
        tf.scale = robcraft::engine::math::Vec3(spec->base_scale.x * s, spec->base_scale.y * s,
                                                spec->base_scale.z * s);
        // Ground on the model's real bottom (bounds), falling back to the spec.
        model = spec->model_path.empty() ? std::shared_ptr<robcraft::renderer::Model>()
                                         : this->app_.model_cache_.get(spec->model_path);
        tf.position.y =
            ground + robcraft::renderer::placement_ground_offset(spec, model.get(), tf.scale);
        if (jitter) tf.rotation = Quaternion::from_euler(0, angle, 0);
    } else {
        tf.scale = robcraft::engine::math::Vec3(s, s, s);
        tf.position.y = ground + 0.5 * s;
    }

    this->app_.world_.add_component<Transform3D>(e, tf);
    this->app_.world_.add_component<Name>(e, Name{std::string(prefix) + "_" + std::to_string(e)});

    bool solid = (spec && !spec->model_path.empty()) ||
                 this->app_.placeable_ == PlaceableType::Bed ||
                 this->app_.placeable_ == PlaceableType::Chair ||
                 this->app_.placeable_ == PlaceableType::Couch ||
                 this->app_.placeable_ == PlaceableType::LightFloor ||
                 this->app_.placeable_ == PlaceableType::Shelf ||
                 this->app_.placeable_ == PlaceableType::Table;
    if (solid) {
        this->app_.world_.add_component<BoxCollider>(
            e, BoxCollider{collider_half_extents(model.get(), tf.scale)});
        t.set_walkable(cx, cz, false);
    }
    this->app_.tools_.mark_modified();
    this->app_.undo_recorder_.record_after_entity(e);
    this->app_.commit_undo("Place");
    this->clear_selection();
    this->add_selection(e);
}

const char* EditorPlacement::prefix_for_type(PlaceableType type) const {
    static const char* kPrefixes[static_cast<int>(PlaceableType::Count)] = {
        // Buildings
        "wall",   // Wall
        "floor",  // Floor
        // Furniture
        "bed",          // Bed
        "chair",        // Chair
        "couch",        // Couch
        "light_floor",  // LightFloor
        "shelf",        // Shelf
        "table",        // Table
        // Nature
        "tree",          // Tree
        "tree_2",        // Tree2
        "tree_3",        // Tree3
        "tree_4",        // Tree4
        "tree_5",        // Tree5
        "pine_1",        // Pine1
        "pine_2",        // Pine2
        "pine_3",        // Pine3
        "pine_4",        // Pine4
        "pine_5",        // Pine5
        "twisted_1",     // Twisted1
        "twisted_2",     // Twisted2
        "twisted_3",     // Twisted3
        "twisted_4",     // Twisted4
        "twisted_5",     // Twisted5
        "dead_1",        // Dead1
        "dead_2",        // Dead2
        "dead_3",        // Dead3
        "dead_4",        // Dead4
        "dead_5",        // Dead5
        "bush",          // Bush
        "bush_2",        // Bush2
        "rock_1",        // Rock1
        "rock_2",        // Rock2
        "rock_3",        // Rock3
        "rock_4",        // Rock4
        "rock_large_1",  // RockLarge1
        "rock_large_2",  // RockLarge2
        "rock_large_3",  // RockLarge3
        // Space
        "moon_rock",        // MoonRock
        "moon_rock_2",      // MoonRock2
        "moon_rock_3",      // MoonRock3
        "moon_rock_large",  // MoonRockLarge
        "space_base",       // SpaceBase
        "geodesic_dome",    // GeodesicDome
        "solar_panel",      // SolarPanel
        // Robots
        "robot_george",  // RobotGeorge
        "robot_leela",   // RobotLeela
        "robot_mike",    // RobotMike
        "robot_stan",    // RobotStan
        // Lights
        "light",  // PointLight
        // Animals
        "cow",    // Cow
        "horse",  // Horse
        "llama",  // Llama
        "pig",    // Pig
        "pug",    // Pug
        "sheep",  // Sheep
        "zebra",  // Zebra
    };
    int idx = static_cast<int>(type);
    if (idx < 0 || idx >= static_cast<int>(PlaceableType::Count)) return "wall";
    return kPrefixes[idx];
}

Entity EditorPlacement::primary_selection() const {
    return this->app_.selection_.empty() ? INVALID_ENTITY : this->app_.selection_.back();
}

bool EditorPlacement::is_selected(Entity e) const {
    return std::find(this->app_.selection_.begin(), this->app_.selection_.end(), e) !=
           this->app_.selection_.end();
}

void EditorPlacement::clear_selection() {
    this->app_.selection_.clear();
}

void EditorPlacement::add_selection(Entity e) {
    if (!this->is_selected(e)) this->app_.selection_.push_back(e);
}

void EditorPlacement::toggle_selection(Entity e) {
    auto it = std::find(this->app_.selection_.begin(), this->app_.selection_.end(), e);
    if (it != this->app_.selection_.end())
        this->app_.selection_.erase(it);
    else
        this->app_.selection_.push_back(e);
}

void EditorPlacement::restore_walkable_under(Entity e) {
    if (!this->app_.world_.has_terrain()) return;
    auto* tf = this->app_.world_.get_component<Transform3D>(e);
    auto* col = this->app_.world_.get_component<BoxCollider>(e);
    if (!tf || !col) return;
    const double eps = 1e-6;
    Vec3 half = col->half_extents;
    int x0, z0, x1, z1;
    this->app_.world_.terrain().world_to_cell(tf->position.x - half.x, tf->position.z - half.z, x0,
                                              z0);
    this->app_.world_.terrain().world_to_cell(tf->position.x + half.x - eps,
                                              tf->position.z + half.z - eps, x1, z1);
    for (int z = z0; z <= z1; ++z) {
        for (int x = x0; x <= x1; ++x) {
            if (this->app_.world_.terrain().in_bounds(x, z))
                this->app_.world_.terrain().set_walkable(x, z, true);
        }
    }
}

void EditorPlacement::delete_selection() {
    if (this->app_.selection_.empty()) return;
    this->app_.undo_recorder_.begin();
    for (Entity e : this->app_.selection_) {
        if (!this->app_.world_.valid(e)) continue;
        this->app_.undo_recorder_.record_before_entity(e);
        this->app_.undo_recorder_.record_before_cells_under(e);
    }
    for (Entity e : this->app_.selection_) {
        if (!this->app_.world_.valid(e)) continue;
        this->restore_walkable_under(e);
        this->app_.world_.destroy_entity(e);
    }
    this->app_.tools_.mark_modified();
    this->app_.commit_undo("Delete");
    this->app_.selection_.clear();
}

void EditorPlacement::handle_robot_placement(double wx, double wz, const char* prefix) {
    this->app_.undo_recorder_.begin();
    if (this->app_.world_.has_terrain()) {
        int cx, cz;
        this->app_.world_.terrain().world_to_cell(wx, wz, cx, cz);
        auto c = this->app_.world_.terrain().cell_center_world(cx, cz);
        wx = c.x;
        wz = c.z;
    }
    const robcraft::renderer::PlacementSpec* spec =
        robcraft::renderer::placement_spec_for_prefix(prefix);
    auto model = (spec && !spec->model_path.empty()) ? this->app_.model_cache_.get(spec->model_path)
                                                     : std::shared_ptr<robcraft::renderer::Model>();
    Entity e =
        robcraft::renderer::create_robot(this->app_.world_, prefix, wx, wz, model.get(), true);
    if (this->app_.world_.has_terrain()) {
        int cx, cz;
        this->app_.world_.terrain().world_to_cell(wx, wz, cx, cz);
        this->app_.undo_recorder_.record_before_cell(cx, cz);
        this->app_.world_.terrain().set_walkable(cx, cz, false);
    }
    this->clear_selection();
    this->add_selection(e);
    this->app_.tools_.mark_modified();
    this->app_.undo_recorder_.record_after_entity(e);
    this->app_.commit_undo("Place robot");
}

}  // namespace robcraft::editor
