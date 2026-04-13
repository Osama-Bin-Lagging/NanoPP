#!/bin/bash
# Install the NanoPnP file-drop watcher as a systemd service on a Jetson
# (or any Linux host).
#
# Usage, as a non-root user that owns the NanoPnP checkout:
#
#     cd ~/NanoPnP
#     sudo scripts/install-watcher.sh
#
# What it does:
#   1. Creates the data directories (~/nanopnp/{incoming,done,failed,work}).
#   2. Renders scripts/nanopnp-watcher.service with the current user and
#      install paths substituted in.
#   3. Copies the rendered unit to /etc/systemd/system/, daemon-reload,
#      enable, start, and tails the journal.
#
# Idempotent: re-run safely to update.

set -euo pipefail

# ── Resolve paths ────────────────────────────────────────

# The user the service should run as — either $SUDO_USER (when called
# via sudo) or whoever is running this script.
RUN_USER="${SUDO_USER:-$USER}"
RUN_HOME=$(getent passwd "$RUN_USER" | cut -d: -f6)

# Install dir = repo root = parent of this script's dir.
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
INSTALL_DIR=$(cd "$SCRIPT_DIR/.." && pwd)

# Data dir = $HOME/nanopnp (the "drop files here" folder tree).
DATA_DIR="$RUN_HOME/nanopnp"

TEMPLATE="$SCRIPT_DIR/nanopnp-watcher.service"
TARGET="/etc/systemd/system/nanopnp-watcher.service"

if [[ ! -f "$TEMPLATE" ]]; then
    echo "error: template not found: $TEMPLATE" >&2
    exit 1
fi

if [[ $EUID -ne 0 ]]; then
    echo "error: run with sudo (need root to install systemd unit)" >&2
    exit 1
fi

echo "  user         = $RUN_USER"
echo "  install dir  = $INSTALL_DIR"
echo "  data dir     = $DATA_DIR"
echo "  unit target  = $TARGET"
echo

# ── Create data directories ──────────────────────────────

install -d -o "$RUN_USER" -g "$RUN_USER" \
    "$DATA_DIR/incoming" \
    "$DATA_DIR/done" \
    "$DATA_DIR/failed" \
    "$DATA_DIR/work"

# ── Render template ──────────────────────────────────────

sed \
    -e "s|@USER@|${RUN_USER}|g" \
    -e "s|@INSTALL_DIR@|${INSTALL_DIR}|g" \
    -e "s|@DATA_DIR@|${DATA_DIR}|g" \
    "$TEMPLATE" > "$TARGET"

chmod 0644 "$TARGET"

# ── Sanity-check python3 + the nanopnp module ────────────

if ! sudo -u "$RUN_USER" -H python3 -c "import nanopnp.watcher" 2>/dev/null; then
    echo "warning: 'python3 -m nanopnp.watcher' is not importable as $RUN_USER." >&2
    echo "         Check that PYTHONPATH=$INSTALL_DIR is correct and that python3 is installed." >&2
fi

# ── Enable + start ───────────────────────────────────────

systemctl daemon-reload
systemctl enable nanopnp-watcher.service
systemctl restart nanopnp-watcher.service

echo
echo "Service installed and running. Logs:"
echo "    journalctl -u nanopnp-watcher -f"
echo
echo "Drop files into: $DATA_DIR/incoming/"
echo "  (the watcher will debounce ~5s after the last file arrives)"
