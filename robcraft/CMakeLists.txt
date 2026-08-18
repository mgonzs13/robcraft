cmake_minimum_required(VERSION 3.10)
project(robcraft
    VERSION 0.1.0
    DESCRIPTION "Lightweight robotics simulator inspired by early-2000s game engineering"
    LANGUAGES CXX
)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Dependencies via FetchContent ──────────────────────────────────────────────

include(FetchContent)

# spdlog — fast C++ logging
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.13.0
)
FetchContent_MakeAvailable(spdlog)

# nlohmann/json — header-only JSON for configuration
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# Catch2 — unit testing
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.6.0
)
FetchContent_MakeAvailable(Catch2)

# Dear ImGui — immediate-mode GUI
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.6
)
FetchContent_MakeAvailable(imgui)

# stb — single-header image loading
FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG 2c980bb59875b0d32144a71867fbdebb2f77cd20
)
FetchContent_GetProperties(stb)
if(NOT stb_POPULATED)
    FetchContent_Populate(stb)
endif()
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${stb_SOURCE_DIR})

# ── System dependencies ────────────────────────────────────────────────────────

find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# ── ROS 2 dependencies (required) ─────────────────────────────────────────────

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Engine core library ────────────────────────────────────────────────────────

add_library(engine_core STATIC
    src/robcraft/engine/core/entity.cpp
    src/robcraft/engine/core/keyboard_control.cpp
    src/robcraft/engine/core/logging.cpp
    src/robcraft/engine/core/random.cpp
    src/robcraft/engine/core/rate_gate.cpp
    src/robcraft/engine/core/name_utils.cpp
    src/robcraft/engine/core/texture_size.cpp
    src/robcraft/engine/ecs/scene_entities.cpp
    src/robcraft/engine/io/file_dialog.cpp
    src/robcraft/engine/collision/raycast.cpp
    src/robcraft/engine/world/world_serializer.cpp
    src/robcraft/engine/world/gravity_step.cpp
    src/robcraft/engine/world/world_tokenizer.cpp
    src/robcraft/engine/world/terrain.cpp
    src/robcraft/engine/world/terrain_mesh.cpp
    src/robcraft/engine/world/terrain_io.cpp
    src/robcraft/engine/math/mat4.cpp
    src/robcraft/engine/math/quaternion.cpp
    src/robcraft/engine/math/frame_conversion.cpp
    src/robcraft/engine/math/cell_range.cpp
    src/robcraft/engine/simulation/simulation_clock.cpp
    src/robcraft/engine/collision/collision.cpp
    src/robcraft/engine/collision/spatial_grid.cpp
    src/robcraft/engine/core/data_path.cpp
    src/robcraft/sensors/lidar/lidar_sensor.cpp
    src/robcraft/sensors/lidar/lidar_raycast.cpp
    src/robcraft/sensors/imu/imu_sensor.cpp
    src/robcraft/sensors/gps/gps_sensor.cpp
    src/robcraft/sensors/magnetometer/magnetometer_sensor.cpp
    src/robcraft/sensors/depth_camera/depth_camera_sensor.cpp
    src/robcraft/sensors/lidar3d/lidar3d_sensor.cpp
)

target_include_directories(engine_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(engine_core PUBLIC
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    GLEW::GLEW
    OpenGL::GL
    ${ament_index_cpp_TARGETS}
)

# ── Renderer library ───────────────────────────────────────────────────────────

add_library(renderer_core STATIC
    src/robcraft/renderer/shader.cpp
    src/robcraft/renderer/mesh.cpp
    src/robcraft/renderer/renderer.cpp
    src/robcraft/renderer/camera.cpp
    src/robcraft/renderer/camera_controls.cpp
    src/robcraft/renderer/pick.cpp
    src/robcraft/renderer/reflection.cpp
    src/robcraft/renderer/sky_render.cpp
    src/robcraft/renderer/fbo.cpp
    src/robcraft/renderer/texture.cpp
    src/robcraft/renderer/texture_pack.cpp
    src/robcraft/renderer/obj_loader.cpp
    src/robcraft/renderer/gltf_loader.cpp
    src/robcraft/renderer/model.cpp
    src/robcraft/renderer/animation_player.cpp
    src/robcraft/renderer/model_paths.cpp
    src/robcraft/renderer/primitive_meshes.cpp
    src/robcraft/renderer/robot_factory.cpp
    src/robcraft/renderer/shader_state.cpp
    src/robcraft/renderer/scene_render.cpp
    src/robcraft/renderer/window_icon.cpp
)

target_include_directories(renderer_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(renderer_core PUBLIC
    engine_core
    stb
    GLEW::GLEW
    glfw
    OpenGL::GL
)

# ── ROS 2 bridge library ─────────────────────────────────────────────────────

add_library(ros2_bridge STATIC
    src/robcraft/ros2/ros2_bridge.cpp
    src/robcraft/ros2/ros2_msg_builders.cpp
)

target_include_directories(ros2_bridge PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(ros2_bridge PUBLIC
    engine_core
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    ${rosgraph_msgs_TARGETS}
    ${builtin_interfaces_TARGETS}
)

# ── Dear ImGui library ─────────────────────────────────────────────────────────

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui PUBLIC
    glfw
    OpenGL::GL
)

# ── Shared ImGui UI helpers (zenity fallback) ────────────────────────────────

add_library(rc_io_ui STATIC
    src/robcraft/engine/io/text_path_dialog.cpp
)

target_include_directories(rc_io_ui PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(rc_io_ui PUBLIC
    imgui
)

# ── Simulator executable ──────────────────────────────────────────────────────

add_executable(robcraft
    src/robcraft/engine/core/main.cpp
    src/robcraft/engine/core/app_menu.cpp
)

target_link_libraries(robcraft PRIVATE engine_core renderer_core rc_io_ui imgui)

target_link_libraries(robcraft PRIVATE ros2_bridge)

# ── Editor executable ──────────────────────────────────────────────────────────

add_executable(robcraft-editor
    src/robcraft/editor/editor_app.cpp
    src/robcraft/editor/editor_dialogs.cpp
    src/robcraft/editor/editor_ui.cpp
    src/robcraft/editor/editor_panels.cpp
    src/robcraft/editor/editor_inspector.cpp
    src/robcraft/editor/editor_view_settings.cpp
    src/robcraft/editor/editor_viewport.cpp
    src/robcraft/editor/editor_tools.cpp
    src/robcraft/editor/editor_place.cpp
    src/robcraft/editor/gizmo_math.cpp
    src/robcraft/editor/editor_gizmo.cpp
    src/robcraft/editor/editor_undo.cpp
    src/robcraft/editor/command/command.cpp
    src/robcraft/editor/command/entity_snapshot.cpp
    src/robcraft/editor/command/world_edit_command.cpp
)

target_include_directories(robcraft-editor PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(robcraft-editor PRIVATE
    engine_core
    renderer_core
    imgui
    rc_io_ui
)

# ── Tests ─────────────────────────────────────────────────────────────────────

add_executable(robcraft-tests
    tests/test_logging.cpp
    tests/test_entity.cpp
    tests/test_math.cpp
    tests/test_frame_conversion.cpp
    tests/test_world.cpp
    tests/test_simulation_clock.cpp
    tests/test_mat4.cpp
    tests/test_differential_drive.cpp
    tests/test_collision.cpp
    tests/test_raycast.cpp
    tests/test_random.cpp
    tests/test_rate_gate.cpp
    tests/test_name_utils.cpp
    tests/test_spatial_grid.cpp
    tests/test_lidar.cpp
    tests/test_imu.cpp
    tests/test_gps.cpp
    tests/test_gravity.cpp
    tests/test_magnetometer.cpp
    tests/test_camera.cpp
    tests/test_depth_camera.cpp
    tests/test_reflection.cpp
    tests/test_lidar3d.cpp
    tests/test_lighting.cpp
    tests/test_sky.cpp
    tests/test_view_camera.cpp
    tests/test_pick.cpp
    tests/test_name.cpp
    tests/test_world_serializer.cpp
    tests/test_terrain.cpp
    tests/test_cell_range.cpp
    tests/test_data_path.cpp
    tests/test_texture.cpp
    tests/test_obj_loader.cpp
    tests/test_gltf_loader.cpp
    tests/test_animal_gltf.cpp
    tests/test_animation_player.cpp
    tests/test_model_paths.cpp
    tests/test_robot_factory.cpp
    tests/test_file_dialog.cpp
    tests/test_gizmo.cpp
    tests/test_undo_stack.cpp
    tests/test_entity_snapshot.cpp
    tests/test_world_edit_command.cpp
    tests/test_app_menu.cpp
    tests/test_keyboard_control.cpp
    tests/test_scene_entities.cpp
    tests/test_ros2_bridge.cpp
    src/robcraft/editor/gizmo_math.cpp
    src/robcraft/editor/command/command.cpp
    src/robcraft/editor/command/entity_snapshot.cpp
    src/robcraft/editor/command/world_edit_command.cpp
)

target_link_libraries(robcraft-tests PRIVATE
    engine_core
    renderer_core
    ros2_bridge
    Catch2::Catch2WithMain
)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
# Tests run with the repo root as CWD: model-path disk-existence checks are
# repo-relative, while colcon's default CTest working dir is build/robcraft.
catch_discover_tests(robcraft-tests
    PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# ── Ament export ──────────────────────────────────────────────────────────────

ament_export_dependencies(
    rclcpp geometry_msgs nav_msgs sensor_msgs tf2_msgs rosgraph_msgs
    builtin_interfaces ament_index_cpp
)

# ── Install ───────────────────────────────────────────────────────────────────

install(TARGETS robcraft robcraft-editor DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
    scripts/launch_robcraft.sh
    scripts/launch_robcraft_editor.sh
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY assets worlds
    DESTINATION share/${PROJECT_NAME}
    PATTERN "untitled.world" EXCLUDE
)

# ── Desktop integration ───────────────────────────────────────────────────────
# GNOME 46+ (mutter) ignores the X11 _NET_WM_ICON window property entirely and
# derives window/app icons from the .desktop file matched via StartupWMClass.
# Ship desktop entries so the logo shows as the app icon on GNOME (alt-tab,
# overview, dock, app menus); the _NET_WM_ICON code still serves the WMs that
# read it (KDE, XFCE, ...).

configure_file(robcraft.desktop.in ${CMAKE_BINARY_DIR}/robcraft.desktop @ONLY)
configure_file(robcraft-editor.desktop.in ${CMAKE_BINARY_DIR}/robcraft-editor.desktop @ONLY)
install(FILES
    ${CMAKE_BINARY_DIR}/robcraft.desktop
    ${CMAKE_BINARY_DIR}/robcraft-editor.desktop
    DESTINATION share/applications
)
# The desktop entries exec the launcher scripts (installed with the targets
# above into lib/robcraft/) instead of the raw binaries: desktop launchers run
# with a bare environment, and the launchers source /opt/ros/humble plus the
# package prefix so ROS 2 shared libraries load and assets resolve through
# ament_index_cpp (AMENT_PREFIX_PATH).
# The logo is shipped inside the package's asset directory
# (share/robcraft/assets/logo.png, via install(DIRECTORY assets ...) above) and
# referenced by absolute path from robcraft.desktop / robcraft-editor.desktop.
# No hicolor copy is needed; the desktop entries point directly at the installed
# asset so the logo resolves without depending on an icon theme or XDG_DATA_DIRS.

# Best-effort: also copy the desktop entries + logo into the current user's
# ~/.local/share so the live GNOME session shows the app icon without needing
# the colcon prefix on XDG_DATA_DIRS. Runs during `colcon build`/install.
install(CODE "
    execute_process(
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/install-desktop.sh ${CMAKE_INSTALL_PREFIX}
        RESULT_VARIABLE _rc
    )
    if(NOT _rc EQUAL 0)
        message(WARNING \"robcraft: install-desktop.sh failed, desktop icon may be missing - see logs\")
    endif()
")

ament_package()
