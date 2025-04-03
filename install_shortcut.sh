#!/bin/bash

# Set script to fail on any error
set -e

# Set variables
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"  # Absolute path to this script's directory
SHORTCUT_SRC="$REPO_DIR/shortcuts/auv_control.desktop"
APPLICATIONS_DEST="$HOME/.local/share/applications"
DESKTOP_DEST="$HOME/Desktop"

# Ensure destinations exist
mkdir -p "$APPLICATIONS_DEST"
mkdir -p "$DESKTOP_DEST"

# Temporary modified shortcut
MODIFIED_SHORTCUT="$REPO_DIR/shortcuts/auv_control_temp.desktop"

# Update paths inside the .desktop file
echo "🔧 Patching shortcut for current system..."
sed \
  -e "s|Exec=.*|Exec=bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run remote_pi_pkg gui_main'|" \
  -e "s|Icon=.*|Icon=$REPO_DIR/assets/icon.png|" \
  "$SHORTCUT_SRC" > "$MODIFIED_SHORTCUT"

# Install the modified shortcut
cp "$MODIFIED_SHORTCUT" "$APPLICATIONS_DEST/auv_control.desktop"
cp "$MODIFIED_SHORTCUT" "$DESKTOP_DEST/auv_control.desktop"

# Make both executable
chmod +x "$APPLICATIONS_DEST/auv_control.desktop"
chmod +x "$DESKTOP_DEST/auv_control.desktop"

# Cleanup temp file
rm "$MODIFIED_SHORTCUT"

echo "✅ Shortcut installed and patched!"
echo "You can now launch the AUV Control GUI from the applications menu or your desktop."
