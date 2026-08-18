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

#include <istream>
#include <stdexcept>
#include <string>

namespace robcraft::engine::world {

using namespace robcraft::engine::world;

/** @brief Token kinds recognized by the .world tokenizer. */
enum class TokenType {
    Ident,
    String,
    Number,
    LBrace,
    RBrace,
    Equals,
    LBracket,
    RBracket,
    Comma,
    End,
};

/** @brief A single token produced by the tokenizer. */
struct Token {
    /** @brief The token kind. */
    TokenType type;
    /** @brief The token's text payload. */
    std::string text;
};

/**
 * @brief Splits a .world stream into tokens, skipping whitespace and '#' comments.
 *
 * Not thread-safe; owns a reference to the input stream.
 */
class Tokenizer {
public:
    /**
     * @brief Constructs a tokenizer over an input stream and reads the first token.
     * @param in The stream to read from.
     */
    Tokenizer(std::istream& in);

    /**
     * @brief Returns the current (next unread) token.
     * @return The current token.
     */
    Token peek() const;

    /**
     * @brief Consumes and returns the current token, advancing to the next.
     * @return The token that was consumed.
     */
    Token next();

    /**
     * @brief Consumes the current token if its type matches.
     * @param type The expected token type.
     * @throws std::runtime_error if the current token type differs.
     */
    void expect(TokenType type);

private:
    /** @brief Reads the next token into the current token slot. */
    void advance();

    /** @brief Skips the remainder of the current line. */
    void skip_line();

    /** @brief Reads a double-quoted string token. */
    std::string read_string();

    /** @brief Reads an identifier token. */
    std::string read_ident();

    /** @brief Reads a numeric literal token. */
    std::string read_number();

    /** @brief The stream being tokenized. */
    std::istream& in_;
    /** @brief The most recently read token. */
    Token current_{TokenType::End, ""};
};

}  // namespace robcraft::engine::world
