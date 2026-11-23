#!/usr/bin/env bash
# Fetch Leo Rover's ERC Mars yard worlds (MIT, Fictionlab) at a pinned commit.
set -euo pipefail
cd "$(dirname "$0")/.."
DEST=assets/gazebo/third_party/leo_simulator-ros2
COMMIT=a8a134a339af32bc96d422b218809bb3a39832f4   # branch jazzy, 2025-04-11
if [ ! -d "$DEST/.git" ]; then
  mkdir -p "$(dirname "$DEST")"
  git clone --filter=blob:none --no-checkout --sparse https://github.com/LeoRover/leo_simulator-ros2.git "$DEST"
fi
git -C "$DEST" sparse-checkout set leo_gz_worlds
git -C "$DEST" fetch --depth 1 origin "$COMMIT"
git -C "$DEST" checkout --detach "$COMMIT"
echo "Mars yard models in $DEST/leo_gz_worlds/models"
