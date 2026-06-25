#!/usr/bin/env bash

set -euo pipefail

USER_NAME="${USER:-ubuntu}"
USER_ID="$(id -u)"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-${USER_ID}}"

mkdir -p "${RUNTIME_DIR}"
chmod 700 "${RUNTIME_DIR}"
export XDG_RUNTIME_DIR="${RUNTIME_DIR}"

if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]]; then
  eval "$(dbus-launch --sh-syntax)"
fi

if [[ -z "${GNOME_KEYRING_CONTROL:-}" ]]; then
  eval "$(gnome-keyring-daemon --start --components=pkcs11,secrets,ssh)"
fi

export SSH_AUTH_SOCK="${SSH_AUTH_SOCK:-${GNOME_KEYRING_CONTROL}/ssh}"

exec /usr/bin/github-desktop "$@"
