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

#include "robcraft/editor/command/command.hpp"

namespace robcraft::editor::command {

void UndoStack::execute(std::unique_ptr<ICommand> cmd) {
    cmd->execute();
    this->undo_.push_back(std::move(cmd));
    this->redo_.clear();
}

void UndoStack::undo() {
    if (this->undo_.empty()) return;
    std::unique_ptr<ICommand> cmd = std::move(this->undo_.back());
    this->undo_.pop_back();
    cmd->undo();
    this->redo_.push_back(std::move(cmd));
}

void UndoStack::redo() {
    if (this->redo_.empty()) return;
    std::unique_ptr<ICommand> cmd = std::move(this->redo_.back());
    this->redo_.pop_back();
    cmd->execute();
    this->undo_.push_back(std::move(cmd));
}

void UndoStack::clear() {
    this->undo_.clear();
    this->redo_.clear();
}

bool UndoStack::can_undo() const {
    return !this->undo_.empty();
}

bool UndoStack::can_redo() const {
    return !this->redo_.empty();
}

const char* UndoStack::undo_label() const {
    return this->undo_.empty() ? nullptr : this->undo_.back()->label();
}

const char* UndoStack::redo_label() const {
    return this->redo_.empty() ? nullptr : this->redo_.back()->label();
}

}  // namespace robcraft::editor::command
