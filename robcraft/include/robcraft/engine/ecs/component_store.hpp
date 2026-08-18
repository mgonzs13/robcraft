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

#include <memory>
#include <stdexcept>
#include <typeindex>
#include <unordered_map>

#include "robcraft/engine/core/entity.hpp"

namespace robcraft::engine::ecs {

using namespace robcraft::engine::ecs;
using namespace robcraft::engine::core;

/** @brief Common interface for component stores of any type. */
class IComponentStore {
public:
    virtual ~IComponentStore() = default;

    /**
     * @brief Removes the component stored for an entity.
     * @param e The entity id.
     */
    virtual void remove(Entity e) = 0;

    /**
     * @brief Checks whether an entity has a component.
     * @param e The entity id.
     * @return True if the entity has a component.
     */
    virtual bool has(Entity e) const = 0;

    /** @brief Removes all stored components. */
    virtual void clear() = 0;
};

/** @brief Typed component storage keyed by entity id. */
template <typename T>
class ComponentStore : public IComponentStore {
public:
    /**
     * @brief Adds or overwrites a component for an entity.
     * @param e The entity id.
     * @param component The component value.
     * @return Pointer to the stored component.
     */
    T* add(Entity e, const T& component) {
        this->components_[e] = component;
        return &this->components_[e];
    }

    /**
     * @brief Removes the component stored for an entity.
     * @param e The entity id.
     */
    void remove(Entity e) override { this->components_.erase(e); }

    /**
     * @brief Returns the stored component for an entity.
     * @param e The entity id.
     * @return Pointer to the component, or nullptr if absent.
     */
    T* get(Entity e) {
        auto it = this->components_.find(e);
        return it != this->components_.end() ? &it->second : nullptr;
    }

    /**
     * @brief Returns the stored component for an entity.
     * @param e The entity id.
     * @return Const pointer to the component, or nullptr if absent.
     */
    const T* get(Entity e) const {
        auto it = this->components_.find(e);
        return it != this->components_.end() ? &it->second : nullptr;
    }

    /**
     * @brief Checks whether an entity has a component.
     * @param e The entity id.
     * @return True if the entity has a component.
     */
    bool has(Entity e) const override { return this->components_.count(e) > 0; }

    /** @brief Removes all stored components. */
    void clear() override { this->components_.clear(); }

    /**
     * @brief Returns the number of stored components.
     * @return The component count.
     */
    size_t size() const { return this->components_.size(); }

    /**
     * @brief Returns an iterator to the first stored component.
     * @return A begin iterator.
     */
    auto begin() { return this->components_.begin(); }

    /**
     * @brief Returns an iterator past the last stored component.
     * @return An end iterator.
     */
    auto end() { return this->components_.end(); }

    /**
     * @brief Returns a const iterator to the first stored component.
     * @return A const begin iterator.
     */
    auto begin() const { return this->components_.begin(); }

    /**
     * @brief Returns a const iterator past the last stored component.
     * @return A const end iterator.
     */
    auto end() const { return this->components_.end(); }

private:
    /** @brief Entity-to-component map. */
    std::unordered_map<Entity, T> components_;
};

}  // namespace robcraft::engine::ecs
