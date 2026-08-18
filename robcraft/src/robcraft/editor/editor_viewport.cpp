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

#include "robcraft/editor/editor_viewport.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include "robcraft/editor/editor_app.hpp"
#include "robcraft/engine/collision/collider.hpp"
#include "robcraft/engine/collision/collision.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/scene_entities.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/math/cell_range.hpp"
#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/engine/world/terrain_mesh.hpp"
#include "robcraft/renderer/shader.hpp"
#include "robcraft/renderer/shader_state.hpp"
#include "robcraft/renderer/sky_render.hpp"

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

EditorViewport::EditorViewport(EditorApp& app) : app_(app) {}

void EditorViewport::render_viewport() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Viewport", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImVec2 size = ImGui::GetContentRegionAvail();
    ImVec2 pos = ImGui::GetCursorScreenPos();
    int vw = (int)size.x, vh = (int)size.y;
    if (vw < 1) vw = 1;
    if (vh < 1) vh = 1;

    this->app_.viewport_x_ = (int)pos.x;
    this->app_.viewport_y_ = (int)pos.y;
    this->app_.viewport_w_ = vw;
    this->app_.viewport_h_ = vh;

    this->render_3d_to_fbo(vw, vh);

    if (this->app_.viewport_fbo_.valid()) {
        ImGui::Image((ImTextureID)(intptr_t)this->app_.viewport_fbo_.color_tex(), size,
                     ImVec2(0, 1), ImVec2(1, 0));
    }

    this->app_.viewport_hovered_ = ImGui::IsItemHovered();
    if (this->app_.gizmo_drag_active_ && ImGui::IsMouseReleased(0))
        this->app_.gizmo_tool_.end_gizmo_drag(false);

    if (this->app_.viewport_hovered_) {
        if (ImGui::IsMouseClicked(0)) {
            if (this->app_.current_tool_ == EditorTool::RaiseTerrain ||
                this->app_.current_tool_ == EditorTool::LowerTerrain ||
                this->app_.current_tool_ == EditorTool::FlattenTerrain ||
                this->app_.current_tool_ == EditorTool::CliffTerrain ||
                this->app_.current_tool_ == EditorTool::WaterTerrain ||
                this->app_.current_tool_ == EditorTool::PaintTerrain) {
                this->app_.brush_stroke_active_ = true;
                this->app_.undo_recorder_.begin();
            } else if (!this->app_.gizmo_tool_.handle_gizmo_press()) {
                switch (this->app_.current_tool_) {
                    case EditorTool::Select:
                        this->handle_mouse_pick((int)pos.x, (int)pos.y, vw, vh);
                        break;
                    case EditorTool::Place:
                        this->app_.placement_.handle_place_press();
                        break;
                    default:
                        break;
                }
            }
        }
        if (ImGui::IsMouseDown(0)) {
            if (this->app_.gizmo_drag_active_)
                this->app_.gizmo_tool_.update_gizmo_drag();
            else
                this->handle_terrain_tool();
        }
    }

    if (ImGui::IsMouseReleased(0)) {
        if (this->app_.brush_stroke_active_) {
            this->app_.brush_stroke_active_ = false;
            this->app_.commit_undo("Terrain edit");
        }
        if (this->app_.current_tool_ == EditorTool::Place)
            this->app_.placement_.handle_place_release();
    }

    const char* hint = "Select: click entity";
    switch (this->app_.current_tool_) {
        case EditorTool::Place:
            hint = "Place: click to place, drag for walls/floors";
            break;
        case EditorTool::RaiseTerrain:
            hint = "Raise: drag";
            break;
        case EditorTool::LowerTerrain:
            hint = "Lower: drag";
            break;
        case EditorTool::FlattenTerrain:
            hint = "Flatten: drag";
            break;
        case EditorTool::CliffTerrain:
            hint = "Cliff: drag";
            break;
        case EditorTool::WaterTerrain:
            hint = "Water: drag to paint, tick Clear to erase";
            break;
        case EditorTool::PaintTerrain:
            hint = "Paint: drag";
            break;
        default:
            break;
    }
    ImGui::GetForegroundDrawList()->AddText(ImVec2(pos.x + 5, pos.y + size.y - 20),
                                            IM_COL32(255, 255, 255, 200), hint);
    ImGui::End();
    ImGui::PopStyleVar();
}

void EditorViewport::render_3d_to_fbo(int vw, int vh) {
    if (!this->app_.viewport_fbo_.valid() || this->app_.viewport_fbo_.width() != vw ||
        this->app_.viewport_fbo_.height() != vh) {
        this->app_.viewport_fbo_.create(vw, vh);
        this->app_.editor_camera_.set_perspective(60.0f, (float)vw / vh, 0.1f, 1000.0f);
    }
    if (!this->app_.viewport_fbo_.valid()) return;

    this->app_.viewport_fbo_.bind();
    glViewport(0, 0, vw, vh);
    glClearColor(0.1f, 0.15f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);

    this->app_.shader_.use();
    robcraft::renderer::draw_sky_background(
        this->app_.shader_, this->app_.editor_camera_.projection_matrix(), this->app_.world_.sky());
    this->app_.shader_.set_uniform("uAlpha", 1.0f);
    this->app_.shader_.set_uniform("uUseModelTexture", 0);
    this->app_.shader_.set_uniform("uProjection",
                                   this->app_.editor_camera_.projection_matrix().ptr());
    this->app_.shader_.set_uniform("uView", this->app_.editor_camera_.view_matrix().ptr());
    this->app_.shader_.set_uniform("uTime", static_cast<float>(glfwGetTime()));
    auto cp = this->app_.editor_camera_.position();
    this->app_.shader_.set_uniform("uCameraPos", static_cast<float>(cp.x), static_cast<float>(cp.y),
                                   static_cast<float>(cp.z));

    auto scene_ctx = this->app_.make_scene_ctx();
    auto draw_entity = [&](Entity e) {
        robcraft::renderer::draw_scene_entity(scene_ctx, this->app_.world_, e, 1.0f,
                                              [&](Entity ee, const std::shared_ptr<Model>& m) {
                                                  return this->app_.editor_skin_matrices(ee, m);
                                              });
    };

    // Sun shadow map: render terrain + entities from the sun's viewpoint.
    Mat4 sun_view_proj;
    bool shadowed = robcraft::renderer::render_shadow_pass(
        this->app_.shader_, this->app_.world_, this->app_.terrain_mesh_, this->app_.editor_camera_,
        this->app_.shadow_fbo_, draw_entity, &sun_view_proj);
    if (!shadowed) this->app_.shader_.set_uniform("uUseShadows", 0);
    this->app_.viewport_fbo_.bind();
    glViewport(0, 0, vw, vh);

    robcraft::renderer::upload_scene_lighting(this->app_.shader_, this->app_.world_.lighting());
    robcraft::renderer::upload_point_lights(this->app_.shader_, this->app_.world_);

    // Planar water reflection: mirrored scene into reflection_fbo_.
    bool have_reflection = false;
    Mat4 refl_view;
    if (this->app_.water_reflection_) {
        robcraft::renderer::WaterReflection refl = robcraft::renderer::render_reflection_pass(
            scene_ctx, this->app_.world_, this->app_.terrain_mesh_, this->app_.editor_camera_,
            this->app_.reflection_fbo_, vw, vh, this->app_.water_mesh_.valid(), draw_entity);
        have_reflection = refl.active;
        refl_view = refl.view;
    }
    // The reflection pass restored the camera view; re-bind the viewport FBO.
    this->app_.viewport_fbo_.bind();
    glViewport(0, 0, vw, vh);

    // Terrain — textured splat (or flat color fallback)
    if (this->app_.has_terrain_mesh_) {
        if (this->app_.world_.terrain().dirty()) {
            this->rebuild_terrain_mesh();
            this->rebuild_grid_mesh();
        }
        auto m = Mat4::from_position_rotation(Vec3(0, 0, 0), Quaternion::identity());
        this->app_.shader_.set_uniform("uModel", m.ptr());
        robcraft::renderer::bind_terrain_textures(
            this->app_.shader_, this->app_.texture_pack_.terrain_albedo,
            this->app_.texture_pack_.terrain_normal, this->app_.texture_pack_.use_splat);
        this->app_.terrain_mesh_.draw();
        this->app_.shader_.set_uniform("uUseTerrainTexture", 0);
    }

    // Grid overlay — cell borders only (no triangle diagonals)
    if (this->app_.show_grid_ && this->app_.grid_mesh_.valid()) {
        glLineWidth(1.5f);
        glDisable(GL_DEPTH_TEST);
        auto m = Mat4::from_position_rotation(Vec3(0, 0, 0), Quaternion::identity());
        this->app_.shader_.set_uniform("uModel", m.ptr());
        this->app_.shader_.set_uniform("uLightDir", 0.0f, 0.0f, 0.0f);
        this->app_.shader_.set_uniform("uLightColor", 0.0f, 0.0f, 0.0f);
        this->app_.shader_.set_uniform("uAmbientColor", 0.25f, 0.25f, 0.25f);
        this->app_.grid_mesh_.draw();
        this->app_.shader_.set_uniform("uAmbientColor", 1.0f, 1.0f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glLineWidth(1.0f);
    }

    // Entities — with lighting (scene lighting from the world)
    robcraft::renderer::upload_scene_lighting(this->app_.shader_, this->app_.world_.lighting());

    for (Entity e : robcraft::engine::ecs::collect_scene_entities(this->app_.world_))
        draw_entity(e);

    // Ghosts, brush previews, and selection/hover highlights
    this->render_previews();

    // Water — translucent per-cell quads, animated + tunable via the Water panel
    if (this->app_.has_terrain_mesh_ && this->app_.water_mesh_.valid()) {
        robcraft::renderer::WaterParams wparams;
        wparams.speed = this->app_.water_speed_;
        wparams.wave_amp = this->app_.water_wave_amp_;
        wparams.specular = this->app_.water_specular_;
        wparams.opacity = this->app_.water_opacity_;
        wparams.foam = this->app_.water_foam_;
        wparams.foam_width = this->app_.water_foam_width_;
        wparams.reflection_strength = this->app_.water_reflection_strength_;
        // Preserve the editor's water-only light override (matches the old block).
        this->app_.shader_.set_uniform("uLightDir", 0.5f, 1.0f, 0.3f);
        robcraft::renderer::draw_water_surface(scene_ctx, this->app_.editor_camera_,
                                               this->app_.water_mesh_, this->app_.reflection_fbo_,
                                               refl_view, have_reflection, wparams);
    }

    // Point light visuals: small wireframe cube + range ring when selected.
    {
        auto* ls = this->app_.world_.store<robcraft::engine::lighting::PointLight>();
        auto* ts = this->app_.world_.store<robcraft::engine::ecs::Transform3D>();
        if (ls && ts && this->app_.selection_box_mesh_.valid()) {
            glLineWidth(1.5f);
            this->app_.shader_.set_uniform("uUseTerrainTexture", 0);
            this->app_.shader_.set_uniform("uUseModelTexture", 0);
            this->app_.shader_.set_uniform("uLightDir", 0.0f, 0.0f, 0.0f);
            this->app_.shader_.set_uniform("uLightColor", 0.0f, 0.0f, 0.0f);
            this->app_.shader_.set_uniform("uPointLightCount", 0);
            this->app_.shader_.set_uniform("uAmbientColor", 1.0f, 1.0f, 1.0f);
            for (auto& [le, light] : *ls) {
                auto* tf = ts->get(le);
                if (!tf) continue;
                auto cm = robcraft::engine::math::Mat4::from_position_rotation(
                              tf->position, robcraft::engine::math::Quaternion::identity()) *
                          robcraft::engine::math::Mat4::scale_matrix(
                              robcraft::engine::math::Vec3(0.4f, 0.4f, 0.4f));
                this->app_.shader_.set_uniform("uModel", cm.ptr());
                this->app_.selection_box_mesh_.draw();
                if (this->app_.placement_.is_selected(le) && this->app_.brush_ring_mesh_.valid()) {
                    auto rm =
                        robcraft::engine::math::Mat4::from_position_rotation(
                            robcraft::engine::math::Vec3(tf->position.x, 0.05f, tf->position.z),
                            robcraft::engine::math::Quaternion::identity()) *
                        robcraft::engine::math::Mat4::scale_matrix(
                            robcraft::engine::math::Vec3(light.range, 1.0f, light.range));
                    this->app_.shader_.set_uniform("uModel", rm.ptr());
                    this->app_.brush_ring_mesh_.draw();
                }
            }
            glLineWidth(1.0f);
        }
    }

    this->app_.viewport_fbo_.unbind();
}

void EditorViewport::render_previews() {
    if (!this->app_.world_.has_terrain()) return;

    double mx, my;
    glfwGetCursorPos(this->app_.window_, &mx, &my);
    bool in_viewport =
        mx >= this->app_.viewport_x_ && mx < this->app_.viewport_x_ + this->app_.viewport_w_ &&
        my >= this->app_.viewport_y_ && my < this->app_.viewport_y_ + this->app_.viewport_h_;

    // Hovered entity (Select tool only), recomputed every frame
    this->app_.hovered_entity_ = (this->app_.current_tool_ == EditorTool::Select && in_viewport &&
                                  !this->app_.gizmo_drag_active_)
                                     ? this->app_.placement_.pick_entity()
                                     : INVALID_ENTITY;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);

    auto scene_ctx = this->app_.make_scene_ctx();
    auto draw_overlay = [&](Entity e, float alpha) {
        robcraft::renderer::draw_scene_entity(scene_ctx, this->app_.world_, e, alpha,
                                              [&](Entity ee, const std::shared_ptr<Model>& m) {
                                                  return this->app_.editor_skin_matrices(ee, m);
                                              });
    };
    for (Entity e : this->app_.selection_) this->draw_selection_box(e);
    this->app_.gizmo_tool_.render_gizmo();
    if (this->app_.hovered_entity_ != INVALID_ENTITY &&
        !this->app_.placement_.is_selected(this->app_.hovered_entity_))
        draw_overlay(this->app_.hovered_entity_, 0.45f);

    if (in_viewport) {
        int cx, cz;
        if (this->app_.current_tool_ == EditorTool::Place &&
            this->app_.placement_.cursor_cell(cx, cz)) {
            Transform3D ghost;
            auto& t = this->app_.world_.terrain();
            auto cc = t.cell_center_world(cx, cz);
            float ground = t.height_at_world(cc.x, cc.z);
            ghost.position = Vec3(cc.x, ground, cc.z);
            if (this->app_.placeable_ == PlaceableType::Wall) {
                CellRange run = this->app_.drag_active_
                                    ? wall_run_cells(this->app_.drag_anchor_x_,
                                                     this->app_.drag_anchor_z_, cx, cz)
                                    : CellRange{cx, cz, cx, cz};
                this->app_.placement_.fill_wall_run_transform(t, run, ghost);
                auto gm = Mat4::from_position_rotation(ghost.position, ghost.rotation) *
                          Mat4::scale_matrix(ghost.scale);
                robcraft::renderer::draw_primitive(scene_ctx, "wall", gm, 0.45f);
                for (const CellRange& j : this->app_.placement_.find_corner_joins(run)) {
                    auto jc = t.cell_center_world(j.x0, j.z0);
                    Transform3D ptf;
                    ptf.position = Vec3(jc.x, t.height_at_world(jc.x, jc.z) + 1.5, jc.z);
                    ptf.scale = Vec3(t.cell_size(), 3.0, t.cell_size());
                    auto pm = Mat4::from_position_rotation(ptf.position, Quaternion::identity()) *
                              Mat4::scale_matrix(ptf.scale);
                    robcraft::renderer::draw_primitive(scene_ctx, "wall", pm, 0.45f);
                }
            } else if (this->app_.placeable_ == PlaceableType::Floor) {
                CellRange rect = this->app_.drag_active_
                                     ? floor_rect_cells(this->app_.drag_anchor_x_,
                                                        this->app_.drag_anchor_z_, cx, cz)
                                     : CellRange{cx, cz, cx, cz};
                this->app_.placement_.fill_floor_rect_transform(t, rect, ghost);
                auto gm = Mat4::from_position_rotation(ghost.position, ghost.rotation) *
                          Mat4::scale_matrix(ghost.scale);
                robcraft::renderer::draw_primitive(scene_ctx, "floor", gm, 0.45f);
            } else {
                // Model ghost via PlacementSpec (stable preview: base scale, no rng draw)
                const char* prefix = this->app_.placement_.prefix_for_type(this->app_.placeable_);
                const robcraft::renderer::PlacementSpec* spec =
                    robcraft::renderer::placement_spec_for_prefix(prefix);
                double s = 1.0;
                if (spec) {
                    ghost.scale = robcraft::engine::math::Vec3(
                        spec->base_scale.x * s, spec->base_scale.y * s, spec->base_scale.z * s);
                    auto model = spec->model_path.empty()
                                     ? std::shared_ptr<robcraft::renderer::Model>()
                                     : this->app_.model_cache_.get(spec->model_path);
                    ghost.position.y = ground + robcraft::renderer::placement_ground_offset(
                                                    spec, model.get(), ghost.scale);
                    // Multi-tile objects preview at the footprint center.
                    if (spec->multi_tile) {
                        auto fp =
                            robcraft::renderer::placement_footprint_cells(spec, t.cell_size());
                        int x0 = cx - fp.first / 2;
                        int z0 = cz - fp.second / 2;
                        auto fc = t.cell_center_world(x0 + fp.first / 2, z0 + fp.second / 2);
                        ghost.position.x = fc.x;
                        ghost.position.z = fc.z;
                        ghost.position.y = t.height_at_world(fc.x, fc.z) +
                                           robcraft::renderer::placement_ground_offset(
                                               spec, model.get(), ghost.scale);
                    }
                } else {
                    ghost.scale = robcraft::engine::math::Vec3(s, s, s);
                    ghost.position.y = ground + 0.5 * s;
                }
                auto gm = Mat4::from_position_rotation(ghost.position, ghost.rotation) *
                          Mat4::scale_matrix(ghost.scale);
                bool drawn = false;
                if (spec && !spec->model_path.empty()) {
                    auto model = this->app_.model_cache_.get(spec->model_path);
                    if (model && model->valid()) {
                        robcraft::renderer::draw_model(
                            scene_ctx, model, INVALID_ENTITY, gm, 0.45f,
                            [&](Entity ee, const std::shared_ptr<Model>& m) {
                                return this->app_.editor_skin_matrices(ee, m);
                            });
                        drawn = true;
                    }
                }
                if (!drawn) {
                    this->app_.shader_.set_uniform("uModel", gm.ptr());
                    this->app_.shader_.set_uniform("uAlpha", 0.45f);
                    this->app_.shader_.set_uniform("uUseModelTexture", 0);
                    robcraft::renderer::fallback_mesh_for_name(this->app_.meshes_, prefix).draw();
                }
            }
        }

        bool brush_tool = this->app_.current_tool_ == EditorTool::RaiseTerrain ||
                          this->app_.current_tool_ == EditorTool::LowerTerrain ||
                          this->app_.current_tool_ == EditorTool::FlattenTerrain ||
                          this->app_.current_tool_ == EditorTool::CliffTerrain ||
                          this->app_.current_tool_ == EditorTool::WaterTerrain ||
                          this->app_.current_tool_ == EditorTool::PaintTerrain;
        if (brush_tool && this->app_.placement_.cursor_cell(cx, cz)) {
            auto& t = this->app_.world_.terrain();
            auto cc = t.cell_center_world(cx, cz);
            float r = (float)((this->app_.brush_radius() + 0.5) * t.cell_size());
            auto m =
                Mat4::from_position_rotation(Vec3(cc.x, cc.y + 0.1, cc.z), Quaternion::identity()) *
                Mat4::scale_matrix(Vec3(r, 1.0, r));
            this->app_.shader_.set_uniform("uModel", m.ptr());
            this->app_.shader_.set_uniform("uAlpha", 0.20f);
            this->app_.brush_disc_mesh_.draw();
            this->app_.shader_.set_uniform("uAlpha", 0.80f);
            this->app_.brush_ring_mesh_.draw();
        }
    }

    glDepthFunc(GL_LESS);
    glDisable(GL_BLEND);
    this->app_.shader_.set_uniform("uAlpha", 1.0f);
}

void EditorViewport::draw_selection_box(Entity e) const {
    auto* tf = this->app_.world_.get_component<Transform3D>(e);
    if (!tf || !this->app_.selection_box_mesh_.valid()) return;

    Vec3 center = tf->position;
    Vec3 size = tf->scale;
    auto* cs = this->app_.world_.store<BoxCollider>();
    if (cs && cs->has(e)) {
        auto aabb = AABB::from_box(tf->position, *cs->get(e), tf->rotation);
        center = (aabb.min + aabb.max) * 0.5;
        size = aabb.max - aabb.min;
    } else {
        auto* nm = this->app_.world_.get_component<Name>(e);
        if (nm) {
            std::string path = draw_model_path_for_name(nm->value);
            if (!path.empty()) {
                auto model = this->app_.model_cache_.get(path);
                if (model && model->valid()) {
                    Vec3 ext = model->bounds_max() - model->bounds_min();
                    size = Vec3(ext.x * tf->scale.x, ext.y * tf->scale.y, ext.z * tf->scale.z);
                }
            }
        }
    }
    auto m = Mat4::from_position_rotation(center, Quaternion::identity()) *
             Mat4::scale_matrix(size * 0.5);
    this->app_.shader_.set_uniform("uModel", m.ptr());
    this->app_.shader_.set_uniform("uAlpha", 1.0f);
    this->app_.shader_.set_uniform("uLightDir", 0.0f, 0.0f, 0.0f);
    this->app_.shader_.set_uniform("uLightColor", 0.0f, 0.0f, 0.0f);
    this->app_.shader_.set_uniform("uAmbientColor", 1.0f, 1.0f, 1.0f);
    this->app_.shader_.set_uniform("uUseModelTexture", 0);
    this->app_.selection_box_mesh_.draw();
    this->app_.shader_.set_uniform("uAmbientColor", 0.4f, 0.42f, 0.45f);
    this->app_.shader_.set_uniform("uLightColor", 1.0f, 0.95f, 0.85f);
    this->app_.shader_.set_uniform("uLightDir", 0.5f, 1.0f, 0.3f);
}

void EditorViewport::handle_mouse_pick(int vp_x, int vp_y, int vp_w, int vp_h) {
    this->app_.viewport_x_ = vp_x;
    this->app_.viewport_y_ = vp_y;
    this->app_.viewport_w_ = vp_w;
    this->app_.viewport_h_ = vp_h;
    this->app_.placement_.handle_select_click();
}

void EditorViewport::handle_terrain_tool() {
    if (this->app_.current_tool_ != EditorTool::RaiseTerrain &&
        this->app_.current_tool_ != EditorTool::LowerTerrain &&
        this->app_.current_tool_ != EditorTool::FlattenTerrain &&
        this->app_.current_tool_ != EditorTool::CliffTerrain &&
        this->app_.current_tool_ != EditorTool::WaterTerrain &&
        this->app_.current_tool_ != EditorTool::PaintTerrain)
        return;
    if (!this->app_.world_.has_terrain()) return;
    int cx, cz;
    if (!this->app_.placement_.cursor_cell(cx, cz)) return;
    // Record every cell the brush may mutate before this frame's edit.
    for (int dz = -this->app_.brush_radius(); dz <= this->app_.brush_radius(); ++dz) {
        for (int dx = -this->app_.brush_radius(); dx <= this->app_.brush_radius(); ++dx) {
            this->app_.undo_recorder_.record_before_cell(cx + dx, cz + dz);
        }
    }
    if (this->app_.current_tool_ == EditorTool::RaiseTerrain)
        this->app_.world_.terrain().raise(cx, cz, this->app_.brush_strength_,
                                          this->app_.brush_radius());
    else if (this->app_.current_tool_ == EditorTool::LowerTerrain)
        this->app_.world_.terrain().lower(cx, cz, this->app_.brush_strength_,
                                          this->app_.brush_radius());
    else if (this->app_.current_tool_ == EditorTool::FlattenTerrain) {
        if (ImGui::IsMouseClicked(0))
            this->app_.flatten_target_ = this->app_.world_.terrain().height_at(cx, cz);
        this->app_.world_.terrain().flatten(cx, cz, this->app_.flatten_target_,
                                            this->app_.brush_radius());
    } else if (this->app_.current_tool_ == EditorTool::CliffTerrain) {
        this->app_.world_.terrain().cliff_to_level(
            cx, cz, static_cast<uint8_t>(this->app_.cliff_target_level_),
            this->app_.brush_radius());
    } else if (this->app_.current_tool_ == EditorTool::WaterTerrain) {
        this->app_.world_.terrain().paint_water(cx, cz, !this->app_.water_clear_,
                                                this->app_.brush_radius());
    } else if (this->app_.current_tool_ == EditorTool::PaintTerrain)
        this->app_.world_.terrain().paint_type(cx, cz, this->app_.paint_type_,
                                               this->app_.brush_radius());
    this->app_.tools_.mark_modified();
}

void EditorViewport::rebuild_grid_mesh() {
    if (!this->app_.world_.has_terrain()) return;
    auto d = build_terrain_grid(this->app_.world_.terrain());
    this->app_.grid_mesh_.upload(d.vertices, d.indices, GL_LINES);
}

void EditorViewport::rebuild_water_mesh() {
    if (!this->app_.world_.has_terrain()) return;
    auto d = build_terrain_water_mesh(this->app_.world_.terrain());
    if (d.vertices.empty()) {
        if (this->app_.water_mesh_.valid()) this->app_.water_mesh_.destroy();
        return;
    }
    this->app_.water_mesh_.upload(d.vertices, d.indices);
}

void EditorViewport::rebuild_terrain_mesh() {
    this->app_.world_.terrain().set_texture_repeat(4.0);
    auto d = build_terrain_mesh(this->app_.world_.terrain());
    this->app_.terrain_mesh_.upload(d.vertices, d.indices, d.weights);
    this->app_.has_terrain_mesh_ = true;
    this->app_.world_.terrain().clear_dirty();
    this->rebuild_water_mesh();
}

}  // namespace robcraft::editor
