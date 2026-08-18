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

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/renderer/mesh.hpp"

namespace robcraft::engine::world {
class World;
}

namespace robcraft::renderer {

class Camera;
class FBO;
class Model;
class ModelCache;
class Shader;

struct PrimitiveMeshes;
struct TexturePack;

using namespace robcraft::engine::core;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;

/** @brief Resources the shared scene draws need. */
struct SceneDrawContext {
    /** @brief The active shader. */
    Shader& shader;
    /** @brief Process-wide model cache. */
    ModelCache& models;
    /** @brief Terrain/building textures. */
    const TexturePack& textures;
    /** @brief Procedural fallback meshes. */
    const PrimitiveMeshes& meshes;
};

/** @brief Returns the joint matrices to skin a skinned model, or null when unskinned.
 *  @param entity The entity being drawn.
 *  @param model The resolved model (non-null only on the model path). */
using SkinMatrices = std::function<const std::vector<Mat4>*(Entity, const std::shared_ptr<Model>&)>;

/** @brief Draws one entity: procedural primitive, model, or fallback mesh.
 *  @param alpha Transparency (1.0 opaque; editor previews use <1.0). */
void draw_scene_entity(const SceneDrawContext& ctx, const World& world, Entity e, float alpha,
                       const SkinMatrices& skin);

/** @brief Draws a procedural primitive by name prefix (wall/floor).
 *  @param prefix Name prefix.
 *  @param m Model matrix.
 *  @param alpha Transparency. */
void draw_primitive(const SceneDrawContext& ctx, const char* prefix, const Mat4& m, float alpha);

/** @brief Draws all submeshes of a model with textures on units 2/3 and optional skinning.
 *  @param e The entity being drawn (passed to the skin callback; may be
 *         INVALID_ENTITY for editor-only model draws).
 *  @param m Model matrix.
 *  @param alpha Transparency. */
void draw_model(const SceneDrawContext& ctx, const std::shared_ptr<Model>& model, Entity e,
                const Mat4& m, float alpha, const SkinMatrices& skin);

/** @brief Returns the fallback mesh for a name prefix.
 *  @param meshes The shared primitive meshes.
 *  @param name Entity name (prefix match).
 *  @return The fallback mesh. */
const Mesh& fallback_mesh_for_name(const PrimitiveMeshes& meshes, const std::string& name);

/** @brief Configuration for the sun-shadow pass. */
struct ShadowConfig {
    /** @brief Shadow map size in pixels. */
    int size = 2048;
    /** @brief Distance of the virtual sun from the world origin. */
    double sun_distance = 100.0;
    /** @brief Shadow near plane. */
    float near_plane = 0.1f;
    /** @brief Shadow far plane. */
    float far_plane = 200.0f;
};

/** @brief Renders the sun-shadow depth map and binds it on texture unit 6.
 *  @param shader The active shader.
 *  @param world The world being rendered.
 *  @param terrain_mesh The terrain mesh to draw into the depth map.
 *  @param cam The viewer camera (projection/view restored after the pass).
 *  @param shadow_fbo Depth-only FBO (created/reused at config.size).
 *  @param draw_entity Callback drawing one scene entity.
 *  @param out_sun_view_proj Receives the sun's projection × view matrix so the
 *         caller can re-bind the shadow map for later passes (e.g. robot cameras).
 *  @return True when a shadow map was rendered and bound.
 *  @note Leaves the caller's viewport/FBO state unchanged; the caller re-binds
 *        its own FBO and viewport after the pass. */
bool render_shadow_pass(Shader& shader, const World& world, const Mesh& terrain_mesh,
                        const Camera& cam, FBO& shadow_fbo,
                        const std::function<void(Entity)>& draw_entity, Mat4* out_sun_view_proj,
                        const ShadowConfig& config = {});

/** @brief Result of the water-reflection pass. */
struct WaterReflection {
    /** @brief Whether a reflection was rendered into the FBO. */
    bool active = false;
    /** @brief The reflected view matrix (identity when inactive). */
    Mat4 view;
};

/** @brief Renders the mirrored scene into the reflection FBO and restores the camera view.
 *  @param ctx Shared draw resources.
 *  @param world The world being rendered.
 *  @param terrain_mesh The terrain mesh to draw.
 *  @param cam The viewer camera (view/projection/position).
 *  @param reflection_fbo The FBO receiving the mirrored scene (sized vw×vh).
 *  @param vw Viewport width in pixels.
 *  @param vh Viewport height in pixels.
 *  @param water_mesh_valid Whether a water surface exists (gates the pass).
 *  @param draw_entity Callback drawing one scene entity.
 *  @return Whether the reflection was rendered and its view matrix.
 *  @note The caller re-binds its own FBO and viewport after the pass. */
WaterReflection render_reflection_pass(const SceneDrawContext& ctx, const World& world,
                                       const Mesh& terrain_mesh, const Camera& cam,
                                       FBO& reflection_fbo, int vw, int vh, bool water_mesh_valid,
                                       const std::function<void(Entity)>& draw_entity);

/** @brief Tunable water appearance, applied in draw_water_surface. */
struct WaterParams {
    /** @brief Water animation speed multiplier. */
    float speed = 0.10f;
    /** @brief Water wave amplitude / gradient contrast. */
    float wave_amp = 1.00f;
    /** @brief Water specular highlight strength. */
    float specular = 0.60f;
    /** @brief Water opacity / alpha. */
    float opacity = 0.72f;
    /** @brief Shoreline foam intensity. */
    float foam = 0.60f;
    /** @brief Shoreline foam band width in meters. */
    float foam_width = 1.50f;
    /** @brief Reflection blend strength (0..1). */
    float reflection_strength = 0.85f;
};

/** @brief Draws the translucent water surface (blend, normal map, optional reflection).
 *  @param ctx Shared draw resources.
 *  @param cam The viewer camera (used for the reflection projection×view matrix).
 *  @param water_mesh The water surface mesh.
 *  @param reflection_fbo The reflection FBO (ignored when have_reflection is false).
 *  @param refl_view Reflection view matrix (identity when have_reflection is false).
 *  @param have_reflection Whether to bind the reflection texture.
 *  @param params Tunable water appearance. */
void draw_water_surface(const SceneDrawContext& ctx, const Camera& cam, const Mesh& water_mesh,
                        const FBO& reflection_fbo, const Mat4& refl_view, bool have_reflection,
                        const WaterParams& params = {});

}  // namespace robcraft::renderer
