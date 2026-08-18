#!/usr/bin/env bash
# Launcher for the RobCraft simulator, used by the installed .desktop entry.
# Desktop launchers run with a bare environment: this script sources the ROS 2
# distribution and the robcraft package prefix so the ROS 2 shared libraries
# and AMENT_PREFIX_PATH (used by ament_index_cpp to resolve assets) reach the
# binary. Works from the colcon install prefix (<prefix>/lib/robcraft/) and
# from the source checkout (<repo>/robcraft/scripts/).
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
if [[ -n "${ROS_DISTRO:-}" ]]; then
    if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        echo "error: /opt/ros/${ROS_DISTRO}/setup.bash not found (ROS_DISTRO=${ROS_DISTRO})" >&2
        exit 1
    fi
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    for setup in /opt/ros/*/setup.bash; do
        if [[ -f "${setup}" ]]; then
            source "${setup}"
            break
        fi
    done
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        echo "error: no ROS 2 distribution found under /opt/ros (set ROS_DISTRO or source a ROS 2 environment)" >&2
        exit 1
    fi
fi

# Installed layout: <prefix>/lib/robcraft/launch_robcraft.sh
PREFIX_CANDIDATE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOCAL_SETUP="${PREFIX_CANDIDATE}/share/robcraft/local_setup.bash"
if [[ -f "${LOCAL_SETUP}" ]]; then
    PREFIX="${PREFIX_CANDIDATE}"
    source "${LOCAL_SETUP}"
    cd "${PREFIX}/share/robcraft"
    BIN="${PREFIX}/lib/robcraft/robcraft"
else
    # Source checkout layout: <repo>/robcraft/scripts/launch_robcraft.sh
    WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
    source "${WORKSPACE_ROOT}/install/setup.bash"
    cd "${WORKSPACE_ROOT}"
    BIN="${WORKSPACE_ROOT}/install/robcraft/lib/robcraft/robcraft"
fi
set -u

exec "${BIN}" "$@"