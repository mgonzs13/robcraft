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

#include "robcraft/renderer/sky_render.hpp"

#include <GL/glew.h>

#include <vector>

#include "robcraft/engine/lighting/sky.hpp"
#include "robcraft/engine/math/mat4.hpp"
#include "robcraft/renderer/mesh.hpp"
#include "robcraft/renderer/shader.hpp"

namespace robcraft::renderer {

using namespace robcraft::engine::lighting;
using namespace robcraft::engine::math;

namespace {

/**
 * @brief Builds the fullscreen triangle mesh (covers the NDC viewport).
 * @return The uploaded mesh.
 */
Mesh make_fullscreen_triangle() {
    std::vector<Vertex> verts(3);
    verts[0] = {-1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                1.0f,  1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    verts[1] = {3.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    verts[2] = {-1.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Mesh mesh;
    mesh.upload(verts, std::vector<GLuint>{0, 1, 2});
    return mesh;
}

}  // namespace

void draw_sky_background(Shader& shader, const Mat4& proj, const Sky& sky, bool mirror_y) {
    static Mesh mesh = make_fullscreen_triangle();
    if (!mesh.valid()) return;

    Mat4 identity;
    identity.set_identity();
    shader.set_uniform("uModel", identity.ptr());
    shader.set_uniform("uView", identity.ptr());
    shader.set_uniform("uProjection", identity.ptr());
    shader.set_uniform("uHasSkin", 0);
    shader.set_uniform("uUseClipPlane", 0);
    shader.set_uniform("uInvProj", proj.inverse().ptr());
    shader.set_uniform("uSkyZenith", static_cast<float>(sky.zenith_color.x),
                       static_cast<float>(sky.zenith_color.y),
                       static_cast<float>(sky.zenith_color.z));
    shader.set_uniform("uSkyHorizon", static_cast<float>(sky.horizon_color.x),
                       static_cast<float>(sky.horizon_color.y),
                       static_cast<float>(sky.horizon_color.z));
    shader.set_uniform("uSkyMirrorY", mirror_y ? 1 : 0);
    shader.set_uniform("uDrawSky", 1);

    glDepthMask(GL_FALSE);
    mesh.draw();
    glDepthMask(GL_TRUE);

    shader.set_uniform("uDrawSky", 0);
}

}  // namespace robcraft::renderer
