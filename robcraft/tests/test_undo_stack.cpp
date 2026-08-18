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

#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>

#include "robcraft/editor/command/command.hpp"

using namespace robcraft::editor::command;
namespace {

/** @brief Test command that flips a bool to track execute/undo calls. */
class FlipCommand : public robcraft::editor::command::ICommand {
public:
    explicit FlipCommand(bool& flag, const char* label) : flag_(flag), label_(label) {}
    void execute() override { this->flag_ = !this->flag_; }
    void undo() override { this->flag_ = !this->flag_; }
    const char* label() const override { return this->label_; }

private:
    bool& flag_;
    const char* label_;
};

}  // namespace

TEST_CASE("UndoStack executes and undoes in order", "[undo]") {
    robcraft::editor::command::UndoStack stack;
    bool a = false;
    bool b = false;

    stack.execute(std::make_unique<FlipCommand>(a, "flip a"));
    stack.execute(std::make_unique<FlipCommand>(b, "flip b"));

    REQUIRE(a);
    REQUIRE(b);
    REQUIRE(stack.can_undo());
    REQUIRE(!stack.can_redo());

    stack.undo();
    REQUIRE(a);
    REQUIRE(!b);

    stack.undo();
    REQUIRE(!a);
    REQUIRE(!b);
    REQUIRE(!stack.can_undo());
}

TEST_CASE("UndoStack redo re-applies in order", "[undo]") {
    robcraft::editor::command::UndoStack stack;
    bool a = false;
    bool b = false;

    stack.execute(std::make_unique<FlipCommand>(a, "flip a"));
    stack.execute(std::make_unique<FlipCommand>(b, "flip b"));
    stack.undo();
    stack.undo();

    REQUIRE(stack.can_redo());
    stack.redo();
    REQUIRE(a);
    REQUIRE(!b);

    stack.redo();
    REQUIRE(a);
    REQUIRE(b);
    REQUIRE(!stack.can_redo());
}

TEST_CASE("UndoStack new command clears redo history", "[undo]") {
    robcraft::editor::command::UndoStack stack;
    bool a = false;
    bool b = false;

    stack.execute(std::make_unique<FlipCommand>(a, "flip a"));
    stack.undo();
    REQUIRE(stack.can_redo());

    stack.execute(std::make_unique<FlipCommand>(b, "flip b"));
    REQUIRE(!stack.can_redo());
}

TEST_CASE("UndoStack clear empties both histories", "[undo]") {
    robcraft::editor::command::UndoStack stack;
    bool a = false;

    stack.execute(std::make_unique<FlipCommand>(a, "flip a"));
    stack.undo();
    stack.clear();

    REQUIRE(!stack.can_undo());
    REQUIRE(!stack.can_redo());
}

TEST_CASE("UndoStack labels expose top commands", "[undo]") {
    robcraft::editor::command::UndoStack stack;
    bool a = false;

    REQUIRE(stack.undo_label() == nullptr);
    stack.execute(std::make_unique<FlipCommand>(a, "place wall"));
    REQUIRE(std::string(stack.undo_label()) == "place wall");
    stack.undo();
    REQUIRE(std::string(stack.redo_label()) == "place wall");
}
