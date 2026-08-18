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

#include "robcraft/renderer/scene_render.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <cstring>

#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/scene_entities.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/world/terrain.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/camera.hpp"
#include "robcraft/renderer/fbo.hpp"
#include "robcraft/renderer/model.hpp"
#include "robcraft/renderer/model_paths.hpp"
#include "robcraft/renderer/primitive_meshes.hpp"
#include "robcraft/renderer/reflection.hpp"
#include "robcraft/renderer/shader.hpp"
#include "robcraft/renderer/shader_state.hpp"
#include "robcraft/renderer/sky_render.hpp"
#include "robcraft/renderer/texture.hpp"
#include "robcraft/renderer/texture_pack.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

namespace {

/** @brief Binds a wall/floor texture pair and enables world-UV mapping.
 *  @param ctx Shared draw resources.
 *  @param prefix "wall" or "floor". */
void bind_world_uv(const SceneDrawContext& ctx, const char* prefix) {
    const Texture* albedo = nullptr;
    const Texture* normal = nullptr;
    if (std::strcmp(prefix, "wall") == 0) {
        albedo = &ctx.textures.wall_albedo;
        normal = &ctx.textures.wall_normal;
    } else if (std::strcmp(prefix, "floor") == 0) {
        albedo = &ctx.textures.floor_albedo;
        normal = &ctx.textures.floor_normal;
    }
    if (albedo && albedo->valid()) {
        bind_model_textures(ctx.shader, *albedo, *normal);
        ctx.shader.set_uniform("uUseWorldUV", 1);
        ctx.shader.set_uniform("uWorldUVScale", 0.4f);
    }
}

}  // namespace

void draw_scene_entity(const SceneDrawContext& ctx, const World& world, Entity e, float alpha,
                       const SkinMatrices& skin) {
    if (world.get_component<PointLight>(e)) return;
    const Transform3D* tf = world.get_component<Transform3D>(e);
    if (!tf) return;
    const Name* name = world.get_component<Name>(e);

    Mat4 m =
        Mat4::from_position_rotation(tf->position, tf->rotation) * Mat4::scale_matrix(tf->scale);

    const PlacementSpec* spec = name ? placement_spec_for_name(name->value) : nullptr;
    if (spec && spec->model_path.empty()) {
        draw_primitive(ctx, spec->name_prefix.c_str(), m, alpha);
        return;
    }

    std::string path = name ? draw_model_path_for_name(name->value) : std::string();
    if (!path.empty()) {
        auto model = ctx.models.get(path);
        if (model && model->valid()) {
            draw_model(ctx, model, e, m, alpha, skin);
            return;
        }
    }

    ctx.shader.set_uniform("uUseModelTexture", 0);
    ctx.shader.set_uniform("uHasSkin", 0);
    ctx.shader.set_uniform("uUseWorldUV", 0);
    ctx.shader.set_uniform("uAlpha", alpha);
    ctx.shader.set_uniform("uModel", m.ptr());
    fallback_mesh_for_name(ctx.meshes, name ? name->value : std::string()).draw();
    ctx.shader.set_uniform("uUseWorldUV", 0);
}

void draw_primitive(const SceneDrawContext& ctx, const char* prefix, const Mat4& m, float alpha) {
    const Mesh* mesh = nullptr;
    if (std::strcmp(prefix, "wall") == 0) {
        mesh = &ctx.meshes.wall;
    } else if (std::strcmp(prefix, "floor") == 0) {
        mesh = &ctx.meshes.wall;  // flat plate
    } else if (std::strcmp(prefix, "tree") == 0) {
        mesh = &ctx.meshes.tree;
    } else if (std::strcmp(prefix, "tree_2") == 0) {
        mesh = &ctx.meshes.pine;
    } else if (std::strcmp(prefix, "bush") == 0) {
        mesh = &ctx.meshes.bush;
    }
    if (!mesh) return;
    ctx.shader.set_uniform("uModel", m.ptr());
    ctx.shader.set_uniform("uAlpha", alpha);
    ctx.shader.set_uniform("uUseModelTexture", 0);
    ctx.shader.set_uniform("uUseWorldUV", 0);
    bind_world_uv(ctx, prefix);
    mesh->draw();
    ctx.shader.set_uniform("uUseWorldUV", 0);
    ctx.shader.set_uniform("uUseModelTexture", 0);
}

void draw_model(const SceneDrawContext& ctx, const std::shared_ptr<Model>& model, Entity e,
                const Mat4& m, float alpha, const SkinMatrices& skin) {
    if (!model) return;
    ctx.shader.set_uniform("uModel", m.ptr());
    ctx.shader.set_uniform("uAlpha", alpha);
    bool skinned = model->skinned();
    if (skinned) {
        const std::vector<Mat4>* matrices = skin(e, model);
        if (matrices && !matrices->empty()) {
            upload_bone_matrices(ctx.shader, *matrices);
            ctx.shader.set_uniform("uHasSkin", 1);
        } else {
            ctx.shader.set_uniform("uHasSkin", 0);
        }
    }
    for (const auto& sm : model->submeshes()) {
        bind_model_textures(ctx.shader, sm.albedo, sm.normal);
        sm.mesh.draw();
    }
    if (skinned) ctx.shader.set_uniform("uHasSkin", 0);
    ctx.shader.set_uniform("uUseModelTexture", 0);
}

const Mesh& fallback_mesh_for_name(const PrimitiveMeshes& meshes, const std::string& name) {
    const Mesh* mesh = &meshes.cube;
    if (name.rfind("wall", 0) == 0 || name.rfind("floor", 0) == 0) {
        mesh = &meshes.wall;
    } else if (name.rfind("tree_2", 0) == 0) {
        mesh = &meshes.pine;
    } else if (name.rfind("tree", 0) == 0) {
        mesh = &meshes.tree;
    } else if (name.rfind("bush", 0) == 0) {
        mesh = &meshes.bush;
    } else if (name.rfind("rock", 0) == 0 || name.rfind("cone", 0) == 0) {
        mesh = &meshes.pyramid;
    }
    return *mesh;
}

bool render_shadow_pass(Shader& shader, const World& world, const Mesh& terrain_mesh,
                        const Camera& cam, FBO& shadow_fbo,
                        const std::function<void(Entity)>& draw_entity, Mat4* out_sun_view_proj,
                        const ShadowConfig& config) {
    const SceneLighting& lt = world.lighting();
    if (!world.has_terrain() || !lt.shadows_enabled) {
        shader.set_uniform("uUseShadows", 0);
        return false;
    }
    const double half = std::max(world.terrain().width(), world.terrain().height()) *
                        world.terrain().cell_size() * 0.75;
    if (!shadow_fbo.valid() || shadow_fbo.width() != config.size) {
        shadow_fbo.create_depth_only(config.size, config.size);
    }
    if (!shadow_fbo.valid()) {
        shader.set_uniform("uUseShadows", 0);
        return false;
    }
    Vec3 sun_pos = Vec3(0, 0, 0) - lt.sun_direction.normalized() * config.sun_distance;
    Mat4 sun_view = Mat4::look_at(sun_pos, Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 sun_proj = Mat4::orthographic(static_cast<float>(-half), static_cast<float>(half),
                                       static_cast<float>(-half), static_cast<float>(half),
                                       config.near_plane, config.far_plane);
    Mat4 sun_view_proj = sun_proj * sun_view;
    if (out_sun_view_proj) *out_sun_view_proj = sun_view_proj;

    shadow_fbo.bind();
    glViewport(0, 0, config.size, config.size);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glClear(GL_DEPTH_BUFFER_BIT);
    shader.use();
    shader.set_uniform("uProjection", sun_proj.ptr());
    shader.set_uniform("uView", sun_view.ptr());
    {
        Mat4 id = Mat4::from_position_rotation(Vec3(0, 0, 0), Quaternion::identity());
        shader.set_uniform("uModel", id.ptr());
        shader.set_uniform("uUseTerrainTexture", 0);
        terrain_mesh.draw();
    }
    for (Entity e : collect_scene_entities(world)) draw_entity(e);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    shadow_fbo.unbind();

    // Bind the depth texture for sampling and restore the camera matrices.
    shader.use();
    glActiveTexture(GL_TEXTURE6);
    glBindTexture(GL_TEXTURE_2D, shadow_fbo.depth_tex());
    shader.set_uniform("uShadowMap", 6);
    shader.set_uniform("uSunViewProj", sun_view_proj.ptr());
    shader.set_uniform("uProjection", cam.projection_matrix().ptr());
    shader.set_uniform("uView", cam.view_matrix().ptr());
    return true;
}

WaterReflection render_reflection_pass(const SceneDrawContext& ctx, const World& world,
                                       const Mesh& terrain_mesh, const Camera& cam,
                                       FBO& reflection_fbo, int vw, int vh, bool water_mesh_valid,
                                       const std::function<void(Entity)>& draw_entity) {
    WaterReflection result;
    if (!world.has_terrain()) return result;
    float water_plane = world.terrain().water_plane_height();
    if (water_plane <= Terrain::WATER_OFF || !water_mesh_valid || cam.position().y <= water_plane) {
        return result;
    }
    result.view = reflected_view(cam.view_matrix(), water_plane);
    if (!reflection_fbo.valid() || reflection_fbo.width() != vw || reflection_fbo.height() != vh) {
        reflection_fbo.create(vw, vh);
    }
    if (!reflection_fbo.valid()) return result;
    std::array<float, 4> clip = reflection_clip_plane(water_plane, 0.05f);

    Shader& shader = ctx.shader;
    reflection_fbo.bind();
    glViewport(0, 0, vw, vh);
    // Sky-colored clear: the gradient sky is drawn over it below, so this is
    // only a fallback if the sky draw ever fails.
    glClearColor(0.7f, 0.8f, 0.9f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shader.use();
    draw_sky_background(shader, cam.projection_matrix(), world.sky(), true);
    shader.set_uniform("uAlpha", 1.0f);
    shader.set_uniform("uUseModelTexture", 0);
    shader.set_uniform("uProjection", cam.projection_matrix().ptr());
    shader.set_uniform("uView", result.view.ptr());
    // Specular on reflected geometry must use the virtual (mirrored) eye
    // position; the real eye position would give wrong highlights.
    Vec3 refl_cam = cam.position();
    refl_cam.y = 2.0f * water_plane - static_cast<float>(refl_cam.y);
    shader.set_uniform("uCameraPos", static_cast<float>(refl_cam.x), static_cast<float>(refl_cam.y),
                       static_cast<float>(refl_cam.z));
    shader.set_uniform("uClipPlane", clip[0], clip[1], clip[2], clip[3]);
    shader.set_uniform("uUseClipPlane", 1);
    {
        Mat4 id = Mat4::from_position_rotation(Vec3(0, 0, 0), Quaternion::identity());
        shader.set_uniform("uModel", id.ptr());
        bind_terrain_textures(shader, ctx.textures.terrain_albedo, ctx.textures.terrain_normal,
                              ctx.textures.use_splat);
        terrain_mesh.draw();
        shader.set_uniform("uUseTerrainTexture", 0);
    }
    for (Entity e : collect_scene_entities(world)) draw_entity(e);
    reflection_fbo.unbind();
    shader.set_uniform("uUseClipPlane", 0);
    shader.set_uniform("uView", cam.view_matrix().ptr());
    Vec3 real = cam.position();
    shader.set_uniform("uCameraPos", static_cast<float>(real.x), static_cast<float>(real.y),
                       static_cast<float>(real.z));
    glViewport(0, 0, vw, vh);
    result.active = true;
    return result;
}

void draw_water_surface(const SceneDrawContext& ctx, const Camera& cam, const Mesh& water_mesh,
                        const FBO& reflection_fbo, const Mat4& refl_view, bool have_reflection,
                        const WaterParams& params) {
    Shader& shader = ctx.shader;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);
    static Texture water_normal = Texture::create_water_normal();
    if (water_normal.valid()) {
        water_normal.bind(GL_TEXTURE5);
        shader.set_uniform("uWaterNormalMap", 5);
    }
    shader.set_uniform("uWaterNormalScale", 2.0f);
    shader.set_uniform("uWaterSpeed", params.speed);
    shader.set_uniform("uWaterWaveAmp", params.wave_amp);
    shader.set_uniform("uWaterSpecular", params.specular);
    shader.set_uniform("uWaterOpacity", params.opacity);
    shader.set_uniform("uWaterShallow", 0.10f, 0.45f, 0.55f);
    shader.set_uniform("uWaterDeep", 0.02f, 0.12f, 0.28f);
    shader.set_uniform("uWaterFoam", params.foam);
    shader.set_uniform("uWaterFoamWidth", params.foam_width);
    shader.set_uniform("uWaterDepth", 2.00f);
    shader.set_uniform("uWaterReflectionStrength", params.reflection_strength);
    shader.set_uniform("uWaterReflectionDistort", 0.04f);
    Mat4 id = Mat4::from_position_rotation(Vec3(0, 0, 0), Quaternion::identity());
    shader.set_uniform("uModel", id.ptr());
    shader.set_uniform("uWater", 1);
    if (have_reflection && reflection_fbo.valid()) {
        reflection_fbo.bind_color(GL_TEXTURE4);
        shader.set_uniform("uWaterReflection", 4);
        shader.set_uniform("uUseReflection", 1);
        Mat4 refl_pv = cam.projection_matrix() * refl_view;
        shader.set_uniform("uReflectionProjView", refl_pv.ptr());
    }
    water_mesh.draw();
    shader.set_uniform("uWater", 0);
    if (have_reflection && reflection_fbo.valid()) {
        shader.set_uniform("uUseReflection", 0);
    }
    glDepthFunc(GL_LESS);
    glDisable(GL_BLEND);
}

}  // namespace robcraft::renderer
