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
#include <vector>

namespace robcraft::editor::command {

/** @brief A reversible operation on the editor world. */
class ICommand {
public:
    /** @brief Destroys the command. */
    virtual ~ICommand() = default;
    /** @brief Applies (or re-applies after undo) the operation. */
    virtual void execute() = 0;
    /** @brief Reverts the operation to its before-state. */
    virtual void undo() = 0;
    /** @brief Short human-readable label for the Edit menu / tooltips. */
    virtual const char* label() const = 0;
};

/** @brief LIFO undo/redo history of editor commands. */
class UndoStack {
public:
    /** @brief Runs a command and pushes it onto the undo history. */
    void execute(std::unique_ptr<ICommand> cmd);
    /** @brief Reverts the most recent command, if any. */
    void undo();
    /** @brief Re-applies the most recently undone command, if any. */
    void redo();
    /** @brief Clears both undo and redo histories. */
    void clear();
    /** @brief Whether undo() can be called. */
    bool can_undo() const;
    /** @brief Whether redo() can be called. */
    bool can_redo() const;
    /** @brief Label of the command undo() would revert, or nullptr. */
    const char* undo_label() const;
    /** @brief Label of the command redo() would re-apply, or nullptr. */
    const char* redo_label() const;

private:
    /** @brief Commands that can be undone, most recent last. */
    std::vector<std::unique_ptr<ICommand>> undo_;
    /** @brief Commands that can be redone, most recent first. */
    std::vector<std::unique_ptr<ICommand>> redo_;
};

}  // namespace robcraft::editor::command
