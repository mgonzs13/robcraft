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
#include <typeindex>
#include <unordered_map>

#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/ecs/component_store.hpp"
#include "robcraft/engine/lighting/scene_lighting.hpp"
#include "robcraft/engine/lighting/sky.hpp"
#include "robcraft/engine/world/terrain.hpp"

namespace robcraft::engine::world {

using namespace robcraft::engine::world;
using namespace robcraft::engine::core;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::lighting;

/** @brief Owns entities, their component stores, and the optional terrain. */
class World {
public:
    /** @brief Creates a new entity and returns its handle. */
    Entity create_entity() { return this->entity_mgr_.create(); }

    /**
     * @brief Creates an entity at a specific id when that id is free.
     * @param e The id to claim.
     * @return The claimed id (which may differ if @p e was already alive).
     */
    Entity create_entity_at(Entity e) { return this->entity_mgr_.create_at(e); }

    /**
     * @brief Destroys an entity and removes all of its components.
     * @param e The entity to destroy.
     */
    void destroy_entity(Entity e) {
        for (auto& [type, store] : this->stores_) {
            store->remove(e);
        }
        this->entity_mgr_.destroy(e);
    }

    /**
     * @brief Checks whether an entity handle is currently alive.
     * @param e The entity to check.
     * @return True if the entity is valid.
     */
    bool valid(Entity e) const { return this->entity_mgr_.valid(e); }

    /**
     * @brief Adds a component to an entity, creating its store if needed.
     * @tparam T Component type.
     * @param e The entity.
     * @param component Initial component value.
     * @return Pointer to the stored component, or null if out of storage.
     */
    template <typename T>
    T* add_component(Entity e, const T& component) {
        auto* store = this->get_or_create_store<T>();
        return store->add(e, component);
    }

    /**
     * @brief Removes a component from an entity if present.
     * @tparam T Component type.
     * @param e The entity.
     */
    template <typename T>
    void remove_component(Entity e) {
        auto* store = this->get_store<T>();
        if (store) store->remove(e);
    }

    /**
     * @brief Retrieves a mutable component from an entity.
     * @tparam T Component type.
     * @param e The entity.
     * @return Pointer to the component, or null if absent.
     */
    template <typename T>
    T* get_component(Entity e) {
        auto* store = this->get_store<T>();
        return store ? store->get(e) : nullptr;
    }

    /**
     * @brief Retrieves a const component from an entity.
     * @tparam T Component type.
     * @param e The entity.
     * @return Pointer to the component, or null if absent.
     */
    template <typename T>
    const T* get_component(Entity e) const {
        auto* store = this->get_store<T>();
        return store ? store->get(e) : nullptr;
    }

    /**
     * @brief Checks whether an entity has a component of the given type.
     * @tparam T Component type.
     * @param e The entity.
     * @return True if the component is present.
     */
    template <typename T>
    bool has_component(Entity e) const {
        auto* store = this->get_store<T>();
        return store ? store->has(e) : false;
    }

    /**
     * @brief Returns the component store for a type, or null if absent.
     * @tparam T Component type.
     * @return Pointer to the store, or null.
     */
    template <typename T>
    ComponentStore<T>* store() {
        return this->get_store<T>();
    }

    /**
     * @brief Returns the const component store for a type, or null if absent.
     * @tparam T Component type.
     * @return Pointer to the store, or null.
     */
    template <typename T>
    const ComponentStore<T>* store() const {
        return this->get_store<T>();
    }

    /** @brief Destroys all entities, clears all component stores, and resets
     *  lighting/sky/gravity to defaults (a cleared world is a fresh world). */
    void clear() {
        this->stores_.clear();
        this->entity_mgr_.reset();
        this->lighting_ = SceneLighting{};
        this->sky_ = Sky{};
        this->gravity_ = 9.81;
    }

    /** @brief Returns the underlying entity manager. */
    const EntityManager& entities() const { return this->entity_mgr_; }

    /** @brief Returns a mutable reference to the terrain. */
    Terrain& terrain() { return this->terrain_; }
    /** @brief Returns a const reference to the terrain. */
    const Terrain& terrain() const { return this->terrain_; }
    /** @brief Whether a non-empty terrain has been set. */
    bool has_terrain() const { return this->terrain_.width() > 0; }
    /**
     * @brief Replaces the current terrain.
     * @param t The new terrain.
     */
    void set_terrain(Terrain t) { this->terrain_ = std::move(t); }

    /** @brief Returns a mutable reference to the scene lighting. */
    SceneLighting& lighting() { return this->lighting_; }
    /** @brief Returns a const reference to the scene lighting. */
    const SceneLighting& lighting() const { return this->lighting_; }
    /**
     * @brief Replaces the scene lighting.
     * @param l The new lighting settings.
     */
    void set_lighting(SceneLighting l) { this->lighting_ = l; }

    /** @brief Returns a mutable reference to the gradient sky settings. */
    Sky& sky() { return this->sky_; }
    /** @brief Returns a const reference to the gradient sky settings. */
    const Sky& sky() const { return this->sky_; }
    /**
     * @brief Replaces the gradient sky settings.
     * @param s The new sky settings.
     */
    void set_sky(Sky s) { this->sky_ = s; }

    /** @brief Returns the world gravity in m/s^2 (Earth default 9.81). */
    double gravity() const { return this->gravity_; }
    /**
     * @brief Sets the world gravity in m/s^2.
     * @param g The new gravity value in m/s^2.
     */
    void set_gravity(double g) { this->gravity_ = g; }

private:
    /**
     * @brief Returns the store for a component type, creating it if absent.
     * @tparam T Component type.
     * @return Pointer to the component store.
     */
    template <typename T>
    ComponentStore<T>* get_or_create_store() {
        auto type = std::type_index(typeid(T));
        auto it = this->stores_.find(type);
        if (it != this->stores_.end()) {
            return static_cast<ComponentStore<T>*>(it->second.get());
        }
        auto* store = new ComponentStore<T>();
        this->stores_[type] = std::unique_ptr<IComponentStore>(store);
        return store;
    }

    /**
     * @brief Returns the mutable store for a component type, or null.
     * @tparam T Component type.
     * @return Pointer to the component store, or null.
     */
    template <typename T>
    ComponentStore<T>* get_store() {
        auto type = std::type_index(typeid(T));
        auto it = this->stores_.find(type);
        return it != this->stores_.end() ? static_cast<ComponentStore<T>*>(it->second.get())
                                         : nullptr;
    }

    /**
     * @brief Returns the const store for a component type, or null.
     * @tparam T Component type.
     * @return Pointer to the component store, or null.
     */
    template <typename T>
    const ComponentStore<T>* get_store() const {
        auto type = std::type_index(typeid(T));
        auto it = this->stores_.find(type);
        return it != this->stores_.end() ? static_cast<const ComponentStore<T>*>(it->second.get())
                                         : nullptr;
    }

    /** @brief Entity lifecycle and handle allocation. */
    EntityManager entity_mgr_;
    /** @brief Component stores keyed by component type. */
    std::unordered_map<std::type_index, std::unique_ptr<IComponentStore>> stores_;
    /** @brief Optional heightmap terrain. */
    Terrain terrain_;
    /** @brief Per-world sun and ambient lighting. */
    SceneLighting lighting_;
    /** @brief Per-world gradient sky. */
    Sky sky_;
    /** @brief Vertical acceleration applied to robots in m/s^2. */
    double gravity_ = 9.81;
};

}  // namespace robcraft::engine::world
