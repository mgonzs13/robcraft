#!/usr/bin/env bash
# Install RobCraft desktop entries + logo into the current user's
# ~/.local/share so the app icon shows in GNOME launchers/overview/dock.
# Works for a colcon dev workspace without needing system-wide install.
#
# Usage:
#   ./scripts/install-desktop.sh [PREFIX]
#
# PREFIX (optional) = package install prefix. Defaults to the first existing
# of: $ROBCRAFT_INSTALL, <repo>/install/robcraft.
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PREFIX="${1:-${ROBCRAFT_INSTALL:-}}"
if [[ -z "${PREFIX}" ]]; then
    for candidate in "${WORKSPACE_ROOT}"/install/robcraft; do
        if [[ -d "${candidate}" ]]; then
            PREFIX="${candidate}"
            break
        fi
    done
fi
if [[ -z "${PREFIX}" || ! -d "${PREFIX}" ]]; then
    echo "error: install prefix not found (set ROBCRAFT_INSTALL or pass PREFIX), got '${PREFIX}'" >&2
    exit 1
fi

APPS_SRC="${PREFIX}/share/applications"
ICON_SRC="${PREFIX}/share/robcraft/assets/logo.png"

APPS_DIR="${HOME}/.local/share/applications"
ICON_DIR="${HOME}/.local/share/robcraft/assets"
CACHE_DIR="${HOME}/.local/share/icons/hicolor"

mkdir -p "${APPS_DIR}" "${ICON_DIR}"

# Overwrite the user's copies on every colcon build/install so stale entries
# (e.g. an Exec= pointing at an older binary or launcher) never linger.
install -m 0644 "${APPS_SRC}/robcraft.desktop" "${APPS_SRC}/robcraft-editor.desktop" "${APPS_DIR}/"
install -m 0644 "${ICON_SRC}" "${ICON_DIR}/logo.png"

# Point the desktop files at the copied logo (rewrite the Icon= line) so the
# entries stay valid even though the install prefix differs at runtime.
for f in "${APPS_DIR}"/robcraft.desktop "${APPS_DIR}"/robcraft-editor.desktop; do
    sed -i "s|^Icon=.*|Icon=${ICON_DIR}/logo.png|" "${f}"
done

# Refresh user icon cache so GNOME picks it up without relogin (best-effort).
if command -v gtk-update-icon-cache >/dev/null 2>&1; then
    gtk-update-icon-cache -q -t "${CACHE_DIR%/icons/hicolor}" 2>/dev/null || true
fi

# Refresh the desktop-entry database so GNOME sees the overwritten entries
# without relogin (best-effort).
if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database -q "${APPS_DIR}" 2>/dev/null || true
fi

echo "Installed desktop entries to ${APPS_DIR}"
echo "Installed logo to ${ICON_DIR}/logo.png"
echo "Log out and back in (or restart GNOME Shell with Alt+F2 'r') to refresh the icon."
