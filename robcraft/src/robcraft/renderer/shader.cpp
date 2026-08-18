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

#include "robcraft/renderer/shader.hpp"

#include <iostream>

namespace robcraft::renderer {

Shader::~Shader() {
    this->destroy();
}

Shader::Shader(Shader&& o) noexcept : program_(o.program_) {
    o.program_ = 0;
}

Shader& Shader::operator=(Shader&& o) noexcept {
    if (this != &o) {
        this->destroy();
        this->program_ = o.program_;
        o.program_ = 0;
    }
    return *this;
}

void Shader::destroy() {
    if (this->program_) {
        glDeleteProgram(this->program_);
        this->program_ = 0;
    }
}

GLuint Shader::compile_stage(GLenum type, const std::string& src) {
    GLuint shader = glCreateShader(type);
    const char* c_str = src.c_str();
    glShaderSource(shader, 1, &c_str, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info[512];
        glGetShaderInfoLog(shader, 512, nullptr, info);
        std::cerr << "Shader compile error: " << info << std::endl;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool Shader::compile(const std::string& vertex_src, const std::string& fragment_src) {
    GLuint vs = this->compile_stage(GL_VERTEX_SHADER, vertex_src);
    GLuint fs = this->compile_stage(GL_FRAGMENT_SHADER, fragment_src);
    if (!vs || !fs) return false;

    this->program_ = glCreateProgram();
    glAttachShader(this->program_, vs);
    glAttachShader(this->program_, fs);
    glLinkProgram(this->program_);

    GLint success;
    glGetProgramiv(this->program_, GL_LINK_STATUS, &success);
    if (!success) {
        char info[512];
        glGetProgramInfoLog(this->program_, 512, nullptr, info);
        std::cerr << "Shader link error: " << info << std::endl;
        glDeleteProgram(this->program_);
        this->program_ = 0;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    if (success) {
        // Bind sampler uniforms to their fixed texture units explicitly.
        // Uninitialized sampler uniforms can trip GL_INVALID_OPERATION at draw
        // time on some drivers (they default to unit 0, conflicting targets).
        glUseProgram(this->program_);
        const struct {
            const char* name;
            int unit;
        } samplers[] = {
            {"uTerrainAlbedo", 0}, {"uTerrainNormal", 1},   {"uModelAlbedo", 2},
            {"uModelNormal", 3},   {"uWaterReflection", 4}, {"uWaterNormalMap", 5},
            {"uShadowMap", 6},
        };
        for (const auto& s : samplers) {
            GLint loc = glGetUniformLocation(this->program_, s.name);
            if (loc >= 0) glUniform1i(loc, s.unit);
        }
    }
    return success;
}

void Shader::use() const {
    glUseProgram(this->program_);
}

GLint Shader::uniform_location(const std::string& name) const {
    return glGetUniformLocation(this->program_, name.c_str());
}

void Shader::set_uniform(const std::string& name, float value) const {
    glUniform1f(this->uniform_location(name), value);
}

void Shader::set_uniform(const std::string& name, int value) const {
    glUniform1i(this->uniform_location(name), value);
}

void Shader::set_uniform(const std::string& name, const float* mat4) const {
    glUniformMatrix4fv(this->uniform_location(name), 1, GL_FALSE, mat4);
}

void Shader::set_uniform(const std::string& name, float x, float y, float z) const {
    glUniform3f(this->uniform_location(name), x, y, z);
}

void Shader::set_uniform(const std::string& name, float x, float y, float z, float w) const {
    glUniform4f(this->uniform_location(name), x, y, z, w);
}

const char* Shader::default_vertex() {
    return R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor;
layout (location = 3) in vec2 aTexCoord;
layout (location = 4) in vec3 aTangent;
layout (location = 5) in vec4 aSplatWeights;
layout (location = 6) in vec4 aJointWeights;
layout (location = 7) in uvec4 aJointIndices;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat4 uBoneMatrices[64];
uniform int uHasSkin;   // 1 = apply skinning
uniform vec4 uClipPlane;   // clipping half-space for the reflection pass
uniform int uUseClipPlane; // 1 = clip below the plane

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorldPos;
out vec2 vTexCoord;
out vec3 vTangent;
out vec4 vSplatWeights;
out vec2 vNdc;

void main() {
    mat4 skin = mat4(1.0);
    if (uHasSkin == 1) {
        skin = aJointWeights.x * uBoneMatrices[aJointIndices.x]
             + aJointWeights.y * uBoneMatrices[aJointIndices.y]
             + aJointWeights.z * uBoneMatrices[aJointIndices.z]
             + aJointWeights.w * uBoneMatrices[aJointIndices.w];
    }
    vec4 worldPos = uModel * skin * vec4(aPos, 1.0);
    vWorldPos = worldPos.xyz;
    mat3 nm;
    if (uHasSkin == 1)
        nm = mat3(uModel) * mat3(skin);
    else
        nm = mat3(transpose(inverse(uModel)));
    vNormal = nm * aNormal;
    vTangent = nm * aTangent;
    vColor = aColor;
    vTexCoord = aTexCoord;
    vSplatWeights = aSplatWeights;
    if (uUseClipPlane == 1)
        gl_ClipDistance[0] = dot(uClipPlane.xyz, worldPos.xyz) + uClipPlane.w;
    else
        gl_ClipDistance[0] = 1.0;
    gl_Position = uProjection * uView * worldPos;
    vNdc = gl_Position.xy;
}
)";
}

const char* Shader::default_fragment() {
    return R"(
#version 330 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorldPos;
in vec2 vTexCoord;
in vec3 vTangent;
in vec4 vSplatWeights;
in vec2 vNdc;

uniform vec3 uLightDir;
uniform vec3 uLightColor;
uniform vec3 uAmbientColor;
uniform float uSunIntensity;        // sun intensity multiplier
uniform float uAmbientIntensity;    // ambient intensity multiplier
uniform int uPointLightCount;       // 0..8 active point lights
uniform vec4 uPointLightPos[8];     // xyz = world position, w = range
uniform vec4 uPointLightColor[8];   // xyz = color, w = intensity
uniform sampler2D uShadowMap;       // sun-space depth texture (unit 6)
uniform mat4 uSunViewProj;          // sun projection * view
uniform int uUseShadows;            // 1 = sample the sun shadow map
uniform float uAlpha;
uniform float uWorldUVScale;        // world-space UV repeats per meter (walls/floors)
uniform int uUseWorldUV;            // 1 = world-space UVs, 0 = vTexCoord
uniform vec3 uCameraPos;
uniform float uTime;
uniform int uDrawSky;            // 1 = fullscreen gradient sky background
uniform vec3 uSkyZenith;         // sky color at the top of the view
uniform vec3 uSkyHorizon;        // sky color at eye level
uniform mat4 uInvProj;           // inverse projection for view-ray reconstruction
uniform int uSkyMirrorY;         // 1 = render from a mirrored camera (water reflection)

uniform sampler2DArray uTerrainAlbedo;   // 5 layers: grass, dirt, rock, sand, snow
uniform sampler2DArray uTerrainNormal;   // 5 layers, same order
uniform sampler2D uModelAlbedo;
uniform sampler2D uModelNormal;
uniform int uUseTerrainTexture;   // 1 = splat, 0 = flat vertex color
uniform int uUseModelTexture;     // 1 = model albedo/normal, 0 = vertex color
uniform int uWater;               // 1 = water surface shading
uniform float uWaterSpeed;        // animation speed multiplier
uniform float uWaterWaveAmp;      // wave amplitude / gradient contrast
uniform float uWaterSpecular;     // specular highlight strength
uniform float uWaterOpacity;      // alpha blend factor
uniform vec3 uWaterShallow;       // shallow water color
uniform vec3 uWaterDeep;          // deep water color
uniform float uWaterFoam;         // shoreline foam intensity
uniform float uWaterFoamWidth;    // foam band width in meters
uniform float uWaterDepth;        // depth (m) at which water reads fully "deep"
uniform sampler2D uWaterReflection;  // planar reflection texture (unit 4)
uniform int uUseReflection;          // 1 = sample uWaterReflection
uniform mat4 uReflectionProjView;    // projection * reflected view
uniform float uWaterReflectionStrength;  // fresnel mix weight for the reflection
uniform float uWaterReflectionDistort;   // wave-normal UV distortion scale
uniform sampler2D uWaterNormalMap;       // tiling ripple normal map (unit 5)
uniform float uWaterNormalScale;         // world-space UV repeats per meter

out vec4 FragColor;

void main() {
    if (uDrawSky == 1) {
        vec4 ndc = uInvProj * vec4(vNdc, 1.0, 1.0);
        vec3 dir = normalize(ndc.xyz / ndc.w);
        float up = (uSkyMirrorY == 1) ? -dir.y : dir.y;
        float factor = clamp(up, 0.0, 1.0);
        FragColor = vec4(mix(uSkyHorizon, uSkyZenith, factor), 1.0);
        return;
    }

    vec3 normal = normalize(vNormal);

    if (uWater == 1) {
        vec2 p = vWorldPos.xz;
        float t = uTime * uWaterSpeed;
        // Three octaves of scrolling waves, each with its own direction and speed.
        float w1 = sin(p.x * 0.8 + t * 1.3) + cos(p.y * 0.7 + t * 1.1);
        float w2 = sin(p.x * 2.1 - t * 1.6 + p.y * 0.4) + cos(p.y * 2.3 + t * 1.4);
        float w3 = sin((p.x + p.y) * 0.35 + t * 0.9) * 0.5;
        float wave = (w1 * 0.35 + w2 * 0.45 + w3 * 0.20) * uWaterWaveAmp;
        // True analytic gradients of the wave sum (drive the normal).
        float ddx = (0.8 * cos(p.x * 0.8 + t * 1.3)) * 0.35
                  + (2.1 * cos(p.x * 2.1 - t * 1.6 + p.y * 0.4)) * 0.45
                  + (0.35 * cos((p.x + p.y) * 0.35 + t * 0.9)) * 0.5 * 0.20;
        float ddz = (-0.7 * sin(p.y * 0.7 + t * 1.1)) * 0.35
                  + (0.4 * cos(p.x * 2.1 - t * 1.6 + p.y * 0.4)
                     - 2.3 * sin(p.y * 2.3 + t * 1.4)) * 0.45
                  + (0.35 * cos((p.x + p.y) * 0.35 + t * 0.9)) * 0.5 * 0.20;
        ddx *= uWaterWaveAmp;
        ddz *= uWaterWaveAmp;
        // Tiling ripple normal map, two layers scrolling in opposite directions.
        vec2 uv0 = p * uWaterNormalScale + t * vec2(0.11, 0.07);
        vec2 uv1 = p * uWaterNormalScale * 1.5 - t * vec2(0.09, 0.13);
        vec3 nm0 = texture(uWaterNormalMap, uv0).rgb * 2.0 - 1.0;
        vec3 nm1 = texture(uWaterNormalMap, uv1).rgb * 2.0 - 1.0;
        vec3 nm = normalize(nm0 + nm1);
        vec3 wn = normalize(vec3(-ddx + nm.x * 0.6, 1.0, -ddz + nm.z * 0.6));
        vec3 viewDir = normalize(uCameraPos - vWorldPos);
        float ndv = max(dot(wn, viewDir), 0.0);
        // Schlick fresnel approximation (F0 ~ 0.02 for water).
        float fres = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);

        // Real depth from mesh UV.v (terrain height) vs the flat water surface height.
        float depth = max(vWorldPos.y - vTexCoord.y, 0.0);
        float depthF = smoothstep(uWaterDepth, 0.0, depth);
        float f = clamp(0.5 + 0.5 * wave, 0.0, 1.0);
        vec3 col = mix(uWaterDeep, uWaterShallow, mix(depthF, f, 0.5));
        if (uUseReflection == 1) {
            vec4 refClip = uReflectionProjView * vec4(vWorldPos, 1.0);
            vec3 refNdc = refClip.xyz / refClip.w;
            vec2 refUv = refNdc.xy * 0.5 + 0.5;
            refUv += wn.xz * uWaterReflectionDistort;
            refUv = clamp(refUv, 0.001, 0.999);
            vec3 refl = texture(uWaterReflection, refUv).rgb;
            col = mix(col, refl, fres * uWaterReflectionStrength);
        } else {
            col = mix(col, vec3(0.7, 0.8, 0.9), fres * 0.4);
        }

        // Shoreline foam: strong near shore (vTexCoord.x = shore distance), animated.
        float shoreF = smoothstep(uWaterFoamWidth, 0.0, vTexCoord.x);
        float foamNoise = 0.5 + 0.5 * sin(p.x * 3.1 + t * 1.7 + sin(p.y * 2.3 - t * 1.2) * 2.0);
        float foam = shoreF * foamNoise * uWaterFoam;
        foam *= 0.5 + 0.5 * smoothstep(0.5, 1.0, wave + 1.0);  // more foam on crests
        col = mix(col, vec3(1.0, 1.0, 1.0), foam);

        // Blinn-Phong specular glint from the directional light.
        vec3 ldir = normalize(uLightDir);
        vec3 halfV = normalize(ldir + viewDir);
        float spec = pow(max(dot(wn, halfV), 0.0), 48.0) * uWaterSpecular;
        col += vec3(spec);

        FragColor = vec4(col, uWaterOpacity);
        return;
    }

    vec3 alb;
    if (uUseTerrainTexture == 1) {
        float w5 = max(1.0 - (vSplatWeights.x + vSplatWeights.y + vSplatWeights.z
                               + vSplatWeights.w), 0.0);
        alb = texture(uTerrainAlbedo, vec3(vTexCoord, 0.0)).rgb * vSplatWeights.x
            + texture(uTerrainAlbedo, vec3(vTexCoord, 1.0)).rgb * vSplatWeights.y
            + texture(uTerrainAlbedo, vec3(vTexCoord, 2.0)).rgb * vSplatWeights.z
            + texture(uTerrainAlbedo, vec3(vTexCoord, 3.0)).rgb * vSplatWeights.w
            + texture(uTerrainAlbedo, vec3(vTexCoord, 4.0)).rgb * w5;
        vec3 nrm = texture(uTerrainNormal, vec3(vTexCoord, 0.0)).rgb * vSplatWeights.x
                 + texture(uTerrainNormal, vec3(vTexCoord, 1.0)).rgb * vSplatWeights.y
                 + texture(uTerrainNormal, vec3(vTexCoord, 2.0)).rgb * vSplatWeights.z
                 + texture(uTerrainNormal, vec3(vTexCoord, 3.0)).rgb * vSplatWeights.w
                 + texture(uTerrainNormal, vec3(vTexCoord, 4.0)).rgb * w5;
        nrm = normalize(nrm * 2.0 - 1.0);
        // Tangent frame: T from vertex, B = cross(N, T). UV v increases along +Z world.
        // Terrain flat ground has T=(1,0,0), N=(0,1,0), so B=(0,0,-1): the normal map's
        // green channel is along -B. Verify terrain bump orientation visually in Task 6.
        vec3 T = normalize(vTangent - vNormal * dot(vNormal, vTangent));
        vec3 B = cross(vNormal, T);
        normal = normalize(T * nrm.x + B * nrm.y + vNormal * nrm.z);
    } else if (uUseModelTexture == 1) {
        vec2 uv = vTexCoord;
        if (uUseWorldUV == 1) {
            vec3 fn = normalize(vNormal);
            vec3 a = abs(fn);
            if (a.y >= a.x && a.y >= a.z)
                uv = vWorldPos.xz * uWorldUVScale;
            else if (a.x >= a.y && a.x >= a.z)
                uv = vWorldPos.zy * uWorldUVScale;
            else
                uv = vWorldPos.xy * uWorldUVScale;
        }
        vec4 texel = texture(uModelAlbedo, uv);
        if (texel.a < 0.5) discard;
        alb = texel.rgb * vColor;
        vec3 nrm = texture(uModelNormal, uv).rgb * 2.0 - 1.0;
        vec3 T = normalize(vTangent - vNormal * dot(vNormal, vTangent));
        vec3 B = cross(vNormal, T);
        normal = normalize(T * nrm.x + B * nrm.y + vNormal * nrm.z);
    } else {
        alb = vColor;
    }

    vec3 viewDir = normalize(uCameraPos - vWorldPos);
    vec3 sun_dir = normalize(uLightDir);

    // Sun shadow: 3x3 PCF against the sun-space depth map.
    float sun_shade = 1.0;
    if (uUseShadows == 1) {
        vec4 sc = uSunViewProj * vec4(vWorldPos, 1.0);
        vec3 sndc = sc.xyz / sc.w;
        vec2 suv = sndc.xy * 0.5 + 0.5;
        if (suv.x >= 0.0 && suv.x <= 1.0 && suv.y >= 0.0 && suv.y <= 1.0 && sndc.z >= -1.0 &&
            sndc.z <= 1.0) {
            float bias = max(0.003 * (1.0 - dot(normal, sun_dir)), 0.001);
            float occluded = 0.0;
            vec2 texel = 1.0 / vec2(textureSize(uShadowMap, 0));
            for (int x = -1; x <= 1; ++x) {
                for (int y = -1; y <= 1; ++y) {
                    float d = texture(uShadowMap, suv + vec2(x, y) * texel).r;
                    // Depth texture stores [0,1] (NDC z mapped), so remap sndc.z.
                    if (d < sndc.z * 0.5 + 0.5 - bias) occluded += 1.0;
                }
            }
            sun_shade = 1.0 - occluded / 9.0;
        }
    }

    vec3 ambient = uAmbientColor * uAmbientIntensity * alb;
    vec3 diffuse = max(dot(normal, sun_dir), 0.0) * uLightColor * uSunIntensity * alb * sun_shade;
    vec3 specular = pow(max(dot(normal, normalize(sun_dir + viewDir)), 0.0), 32.0) *
                    uLightColor * uSunIntensity;

    for (int i = 0; i < uPointLightCount && i < 8; ++i) {
        vec3 to_light = uPointLightPos[i].xyz - vWorldPos;
        float dist = length(to_light);
        float att = pow(clamp(1.0 - dist / max(uPointLightPos[i].w, 1e-4), 0.0, 1.0), 2.0);
        vec3 ldir = to_light / max(dist, 1e-4);
        diffuse += att * uPointLightColor[i].xyz * uPointLightColor[i].w * alb *
                   max(dot(normal, ldir), 0.0);
        specular += att * uPointLightColor[i].xyz * uPointLightColor[i].w *
                    pow(max(dot(normal, normalize(ldir + viewDir)), 0.0), 32.0);
    }

    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, uAlpha);
}
)";
}

}  // namespace robcraft::renderer
