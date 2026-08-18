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
#include <filesystem>
#include <fstream>
#include <system_error>
#include <vector>

#include "robcraft/engine/core/data_path.hpp"

using namespace robcraft::engine::core;
namespace fs = std::filesystem;

namespace {

/**
 * @brief Removes temp files/directories created by a test when the test exits.
 */
struct TempCleanup {
    std::vector<fs::path> paths;
    ~TempCleanup() {
        for (const auto& p : this->paths) {
            std::error_code ec;
            fs::remove_all(p, ec);
        }
    }
};

}  // namespace

TEST_CASE("absolute paths pass through unchanged", "[data_path]") {
    TempCleanup cleanup;
    fs::path p = fs::temp_directory_path() / "rc_abs_check.tmp";
    cleanup.paths.push_back(p);
    std::ofstream(p) << "x";
    REQUIRE(robcraft::engine::core::resolve_data_path(p.string()) == p.string());
}

TEST_CASE("cwd-relative existing file passes through unchanged", "[data_path]") {
    TempCleanup cleanup;
    fs::path f = fs::current_path() / "rc_cwd_check.tmp";
    cleanup.paths.push_back(f);
    std::ofstream(f) << "x";
    std::string rel = "rc_cwd_check.tmp";
    REQUIRE(robcraft::engine::core::resolve_data_path(rel) == rel);
}

TEST_CASE("missing cwd-relative path resolves against explicit share root", "[data_path]") {
    TempCleanup cleanup;
    fs::path root = fs::temp_directory_path() / "rc_share_root_test";
    fs::create_directories(root / "rc_nonexistent");
    fs::path f = root / "rc_nonexistent" / "box.obj";
    cleanup.paths.push_back(root);
    std::ofstream(f) << "x";

    REQUIRE(robcraft::engine::core::resolve_data_path("rc_nonexistent/box.obj", root.string()) ==
            f.string());
}

TEST_CASE("path missing everywhere returns the original string", "[data_path]") {
    REQUIRE(robcraft::engine::core::resolve_data_path("definitely_missing_rc_asset.tmp", "") ==
            "definitely_missing_rc_asset.tmp");
}

TEST_CASE("empty share root leaves relative path untouched", "[data_path]") {
    REQUIRE(robcraft::engine::core::resolve_data_path("assets/models/box.obj", "") ==
            "assets/models/box.obj");
}
