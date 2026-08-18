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

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/**
 * @brief Maps a texture size in pixels to a combo index.
 * @param size Texture size in pixels (256, 512, or 1024).
 * @return Combo index: 256->0, 512->1, 1024->2, anything else->0.
 */
int texture_size_to_index(int size);

/**
 * @brief Maps a combo index to a texture size in pixels.
 * @param index Combo index.
 * @return Texture size: 0->256, 1->512, 2->1024, anything else->256.
 */
int index_to_texture_size(int index);

}  // namespace robcraft::engine::core
