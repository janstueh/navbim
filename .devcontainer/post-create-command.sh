#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Enable autocomplete for user
cp /etc/skel/.bashrc ~/

# Create XDG_RUNTIME_DIR required by Wayland/D-Bus tools (e.g. RViz2)
mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Check if srv folder exists
if [ -d "$ROOT_SRV" ]; then
    # Setup Nav2 web app
    for dir in $OVERLAY_WS/src/navbim/.devcontainer/caddy/srv/*; \
        do if [ -d "$dir" ]; then ln -s "$dir" $ROOT_SRV; fi done
fi
