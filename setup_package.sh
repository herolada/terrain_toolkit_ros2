#!/usr/bin/env bash
set -e  # exit on error

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parent directory of the script
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

REPO_URL="https://github.com/aleskucera/terrain_toolkit.git"
CLONE_DIR="$PARENT_DIR/terrain_toolkit"

# Clone repo if it doesn't exist
if [ ! -d "$CLONE_DIR" ]; then
    echo "Cloning repository into $CLONE_DIR..."
    git clone "$REPO_URL" "$CLONE_DIR"
else
    echo "Repository already exists at $CLONE_DIR"
fi

# Target directory for symlink
TARGET_DIR="$SCRIPT_DIR/terrain_toolkit_ros2"
mkdir -p "$TARGET_DIR"

# Source folder inside cloned repo
SOURCE_DIR="$CLONE_DIR/src/terrain_toolkit"

# Symlink path
LINK_PATH="$TARGET_DIR/terrain_toolkit"

# Create symlink (overwrite if exists)
if [ -L "$LINK_PATH" ] || [ -e "$LINK_PATH" ]; then
    echo "Removing existing link or folder at $LINK_PATH"
    rm -rf "$LINK_PATH"
fi

echo "Creating symlink..."
ln -s "$SOURCE_DIR" "$LINK_PATH"

# Root directory = one level above PARENT_DIR
ROOT_DIR="$(dirname "$PARENT_DIR")"

set -e

echo "Using ROOT_DIR=$ROOT_DIR"

# 2. Move pyproject.toml and README.md
cp "$CLONE_DIR/pyproject.toml" "$ROOT_DIR/"
cp "$CLONE_DIR/README.md" "$ROOT_DIR/"

cd "$ROOT_DIR"

# 3. uv sync
uv sync

# 4. activate venv
source .venv/bin/activate

# get pip3
python3 -m ensurepip --upgrade

# Check for required Python packages
MISSING_PACKAGES=()

for pkg in pyyaml typing_extensions transforms3d; do
    python3 -c "import $pkg" 2>/dev/null || MISSING_PACKAGES+=("$pkg")
done

if [ ${#MISSING_PACKAGES[@]} -ne 0 ]; then
    echo "Installing missing packages: ${MISSING_PACKAGES[*]}"
    pip3 install "${MISSING_PACKAGES[@]}"
else
    echo "All required Python packages are already installed"
fi

# 7. source ROS
source /opt/ros/kilted/setup.bash

# 8. colcon build
read -p "Colcon build the 'terrain_toolkit_ros2' package? Press Enter to continue..."
colcon build --symlink-install --packages-select terrain_toolkit_ros2

read -p "Launch terrain_toolkit? Press Enter to continue..."
source install/setup.bash
source .venv/bin/activate
ros2 launch terrain_toolkit_ros2 terrain_toolkit_node.launch.py

echo "All steps completed successfully."
