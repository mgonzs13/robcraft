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

#include "robcraft/engine/core/texture_size.hpp"

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

int texture_size_to_index(int size) {
    if (size == 512) return 1;
    if (size == 1024) return 2;
    return 0;
}

int index_to_texture_size(int index) {
    if (index == 1) return 512;
    if (index == 2) return 1024;
    return 256;
}

}  // namespace robcraft::engine::core
