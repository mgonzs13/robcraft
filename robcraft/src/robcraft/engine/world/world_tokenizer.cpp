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

#include "robcraft/engine/world/world_tokenizer.hpp"

#include <cctype>
#include <string>

namespace robcraft::engine::world {

using namespace robcraft::engine::world;

Tokenizer::Tokenizer(std::istream& in) : in_(in) {
    this->advance();
}

Token Tokenizer::peek() const {
    return this->current_;
}

Token Tokenizer::next() {
    Token t = this->current_;
    this->advance();
    return t;
}

void Tokenizer::expect(TokenType type) {
    if (this->current_.type != type) {
        throw std::runtime_error("Expected token type " + std::to_string(static_cast<int>(type)));
    }
    this->advance();
}

void Tokenizer::advance() {
    while (this->in_.good()) {
        int c = this->in_.get();
        if (c == EOF || c == -1) {
            this->current_ = {TokenType::End, ""};
            return;
        }
        if (std::isspace(c)) continue;
        if (c == '#') {
            this->skip_line();
            continue;
        }
        if (c == '{') {
            this->current_ = {TokenType::LBrace, "{"};
            return;
        }
        if (c == '}') {
            this->current_ = {TokenType::RBrace, "}"};
            return;
        }
        if (c == '=') {
            this->current_ = {TokenType::Equals, "="};
            return;
        }
        if (c == '[') {
            this->current_ = {TokenType::LBracket, "["};
            return;
        }
        if (c == ']') {
            this->current_ = {TokenType::RBracket, "]"};
            return;
        }
        if (c == ',') {
            this->current_ = {TokenType::Comma, ","};
            return;
        }
        if (c == '"') {
            this->current_ = {TokenType::String, this->read_string()};
            return;
        }
        if (std::isdigit(c) || c == '-' || c == '+') {
            this->in_.unget();
            this->current_ = {TokenType::Number, this->read_number()};
            return;
        }
        if (std::isalpha(c) || c == '_') {
            this->in_.unget();
            this->current_ = {TokenType::Ident, this->read_ident()};
            return;
        }
    }
    this->current_ = {TokenType::End, ""};
}

void Tokenizer::skip_line() {
    std::string dummy;
    std::getline(this->in_, dummy);
}

std::string Tokenizer::read_string() {
    std::string s;
    int c;
    while ((c = this->in_.get()) != '"' && c != EOF && c != -1) s += static_cast<char>(c);
    return s;
}

std::string Tokenizer::read_ident() {
    std::string s;
    int c;
    while ((c = this->in_.peek()) != EOF && c != -1 && (std::isalnum(c) || c == '_'))
        s += static_cast<char>(this->in_.get());
    return s;
}

std::string Tokenizer::read_number() {
    std::string s;
    int c;
    while ((c = this->in_.peek()) != EOF && c != -1 &&
           (std::isdigit(c) || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E'))
        s += static_cast<char>(this->in_.get());
    return s;
}

}  // namespace robcraft::engine::world
