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

#include "robcraft/engine/core/entity.hpp"

#include <cstddef>
#include <queue>
#include <unordered_set>

namespace robcraft::engine::core {

using namespace robcraft::engine::core;

Entity EntityManager::create() {
    if (!this->free_list_.empty()) {
        Entity id = this->free_list_.front();
        this->free_list_.pop();
        this->alive_.insert(id);
        return id;
    }
    Entity id = ++this->next_id_;
    this->alive_.insert(id);
    return id;
}

Entity EntityManager::create_at(Entity e) {
    if (e == INVALID_ENTITY) return this->create();
    if (this->alive_.count(e)) return e;
    // If the id is on the free list, pull it off so a later create() cannot
    // hand it out again. Rebuild the queue without e (queue has no erase).
    std::queue<Entity> kept;
    while (!this->free_list_.empty()) {
        Entity f = this->free_list_.front();
        this->free_list_.pop();
        if (f != e) kept.push(f);
    }
    this->free_list_ = std::move(kept);
    if (e > this->next_id_) this->next_id_ = e;
    this->alive_.insert(e);
    return e;
}

void EntityManager::destroy(Entity e) {
    if (e == INVALID_ENTITY) return;
    if (e > this->next_id_) return;
    this->alive_.erase(e);
    this->free_list_.push(e);
}

bool EntityManager::valid(Entity e) const {
    return this->alive_.count(e) > 0;
}

void EntityManager::reset() {
    this->next_id_ = 0;
    this->free_list_ = {};
    this->alive_.clear();
}

size_t EntityManager::max_allocated() const {
    return this->next_id_;
}

}  // namespace robcraft::engine::core
