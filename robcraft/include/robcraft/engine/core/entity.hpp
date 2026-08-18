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

#include <cstddef>
#include <cstdint>
#include <queue>
#include <unordered_set>

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

/** @brief Opaque 32-bit entity identifier. */
using Entity = uint32_t;

/** @brief Sentinel entity id meaning "no entity". */
constexpr Entity INVALID_ENTITY = 0;

/** @brief Manages entity creation and destruction. */
class EntityManager {
public:
    /**
     * @brief Creates a new entity, reusing a freed id when available.
     * @return The new entity id.
     */
    Entity create();

    /**
     * @brief Creates an entity at a specific id when that id is free.
     * Used by undo/redo restore so entities return to their recorded id.
     * @param e The id to claim.
     * @return The claimed id (which may differ if @p e was already alive).
     */
    Entity create_at(Entity e);

    /**
     * @brief Destroys an entity and frees its id for reuse.
     * @param e The entity id to destroy.
     */
    void destroy(Entity e);

    /**
     * @brief Checks whether an entity is currently alive.
     * @param e The entity id.
     * @return True if the entity is alive.
     */
    bool valid(Entity e) const;

    /** @brief Destroys all entities and resets the id allocator. */
    void reset();

    /**
     * @brief Returns the highest id ever allocated.
     * @return The peak allocated id.
     */
    size_t max_allocated() const;

private:
    /** @brief Next id to allocate. */
    Entity next_id_ = 0;
    /** @brief Ids freed and available for reuse. */
    std::queue<Entity> free_list_;
    /** @brief Currently alive entity ids. */
    std::unordered_set<Entity> alive_;
};

}  // namespace robcraft::engine::core
