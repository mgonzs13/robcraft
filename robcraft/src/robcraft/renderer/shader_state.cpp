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

#include "robcraft/renderer/shader_state.hpp"

#include <GL/glew.h>

#include <cstring>

#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/lighting/point_light.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/renderer/shader.hpp"
#include "robcraft/renderer/texture.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;
using namespace robcraft::engine::world;

void upload_scene_lighting(const Shader& shader, const SceneLighting& lighting) {
    shader.set_uniform("uLightDir", static_cast<float>(lighting.sun_direction.x),
                       static_cast<float>(lighting.sun_direction.y),
                       static_cast<float>(lighting.sun_direction.z));
    shader.set_uniform("uLightColor", lighting.sun_color.x, lighting.sun_color.y,
                       lighting.sun_color.z);
    shader.set_uniform("uAmbientColor", lighting.ambient_color.x, lighting.ambient_color.y,
                       lighting.ambient_color.z);
    shader.set_uniform("uSunIntensity", lighting.sun_intensity);
    shader.set_uniform("uAmbientIntensity", lighting.ambient_intensity);
    shader.set_uniform("uUseShadows", lighting.shadows_enabled ? 1 : 0);
}

void upload_point_lights(const Shader& shader, const World& world) {
    auto* ls = world.store<PointLight>();
    auto* ts = world.store<Transform3D>();
    float pos[8 * 4];
    float col[8 * 4];
    int count = 0;
    if (ls) {
        for (auto& [e, light] : *ls) {
            if (count >= 8) break;
            const Transform3D* tf = ts ? ts->get(e) : nullptr;
            pos[count * 4 + 0] = tf ? static_cast<float>(tf->position.x) : 0.0f;
            pos[count * 4 + 1] = tf ? static_cast<float>(tf->position.y) : 0.0f;
            pos[count * 4 + 2] = tf ? static_cast<float>(tf->position.z) : 0.0f;
            pos[count * 4 + 3] = light.range;
            col[count * 4 + 0] = light.color.x;
            col[count * 4 + 1] = light.color.y;
            col[count * 4 + 2] = light.color.z;
            col[count * 4 + 3] = light.intensity;
            ++count;
        }
    }
    shader.set_uniform("uPointLightCount", count);
    if (count > 0) {
        GLint loc_p = glGetUniformLocation(shader.id(), "uPointLightPos");
        GLint loc_c = glGetUniformLocation(shader.id(), "uPointLightColor");
        glUniform4fv(loc_p, count, pos);
        glUniform4fv(loc_c, count, col);
    }
}

void upload_bone_matrices(const Shader& shader, const std::vector<Mat4>& matrices) {
    float data[64 * 16];
    for (int i = 0; i < 64; ++i) {
        std::memset(&data[i * 16], 0, 16 * sizeof(float));
        data[i * 16 + 0] = 1.0f;
        data[i * 16 + 5] = 1.0f;
        data[i * 16 + 10] = 1.0f;
        data[i * 16 + 15] = 1.0f;
    }
    for (size_t i = 0; i < matrices.size() && i < 64; ++i) {
        std::memcpy(&data[i * 16], matrices[i].ptr(), 16 * sizeof(float));
    }
    GLint loc = glGetUniformLocation(shader.id(), "uBoneMatrices");
    glUniformMatrix4fv(loc, 64, GL_FALSE, data);
}

void bind_terrain_textures(const Shader& shader, const Texture& albedo, const Texture& normal,
                           bool use_splat) {
    shader.set_uniform("uUseTerrainTexture", use_splat ? 1 : 0);
    if (albedo.valid()) {
        albedo.bind(GL_TEXTURE0);
        shader.set_uniform("uTerrainAlbedo", 0);
    }
    if (normal.valid()) {
        normal.bind(GL_TEXTURE1);
        shader.set_uniform("uTerrainNormal", 1);
    }
}

void bind_model_textures(const Shader& shader, const Texture& albedo, const Texture& normal) {
    shader.set_uniform("uUseModelTexture", albedo.valid() ? 1 : 0);
    if (albedo.valid()) {
        albedo.bind(GL_TEXTURE2);
        shader.set_uniform("uModelAlbedo", 2);
    }
    if (normal.valid()) {
        normal.bind(GL_TEXTURE3);
        shader.set_uniform("uModelNormal", 3);
    } else if (albedo.valid()) {
        static Texture neutral = Texture::create_neutral_normal();
        if (neutral.valid()) {
            neutral.bind(GL_TEXTURE3);
            shader.set_uniform("uModelNormal", 3);
        }
    }
}

void set_water_defaults(const Shader& shader) {
    static Texture water_normal = Texture::create_water_normal();
    if (water_normal.valid()) {
        water_normal.bind(GL_TEXTURE5);
        shader.set_uniform("uWaterNormalMap", 5);
    }
    shader.set_uniform("uWaterSpeed", 0.10f);
    shader.set_uniform("uWaterWaveAmp", 1.00f);
    shader.set_uniform("uWaterSpecular", 0.60f);
    shader.set_uniform("uWaterOpacity", 0.72f);
    shader.set_uniform("uWaterShallow", 0.10f, 0.45f, 0.55f);
    shader.set_uniform("uWaterDeep", 0.02f, 0.12f, 0.28f);
    shader.set_uniform("uWaterFoam", 0.60f);
    shader.set_uniform("uWaterFoamWidth", 1.50f);
    shader.set_uniform("uWaterDepth", 2.00f);
    shader.set_uniform("uWaterNormalScale", 2.0f);
    shader.set_uniform("uWaterReflectionStrength", 0.85f);
    shader.set_uniform("uWaterReflectionDistort", 0.04f);
    shader.set_uniform("uUseReflection", 0);
    shader.set_uniform("uUseClipPlane", 0);
    shader.set_uniform("uClipPlane", 0.0f, 1.0f, 0.0f, 0.0f);
}

}  // namespace robcraft::renderer
