#!/bin/bash

# Absolute paths
WORKSPACE_DIR="$HOME/RemotePiRos2"
LAUNCH_SCRIPT="$WORKSPACE_DIR/launch_gui.sh"
ICON_PATH="$WORKSPACE_DIR/assets/icon.png"
DESKTOP_FILE="$HOME/.local/share/applications/auv-control.desktop"

# Step 1: Create launch script
echo "Creating launch script at $LAUNCH_SCRIPT"
cat << EOF > "$LAUNCH_SCRIPT"
#!/bin/bash
export QT_QPA_PLATFORM=xcb
source /opt/ros/jazzy/setup.bash
source $WORKSPACE_DIR/install/setup.bash
ros2 run remote_pi_pkg gamepad_mapper &
ros2 run remote_pi_pkg auv_control
EOF

chmod +x "$LAUNCH_SCRIPT"

# Step 2: Create .desktop entry
echo "Creating .desktop shortcut at $DESKTOP_FILE"
cat << EOF > "$DESKTOP_FILE"
[Desktop Entry]
Name=AUV Control Interface
Comment=Touchscreen GUI for controlling the AUV
Exec=$LAUNCH_SCRIPT
Icon=$ICON_PATH
Terminal=false
Type=Application
Categories=Utility;
EOF

chmod +x "$DESKTOP_FILE"

# Optional: Link to Desktop for quick access
ln -sf "$DESKTOP_FILE" "$HOME/Desktop/auv-control.desktop"

echo "Shortcut installed! You may need to right-click the icon and allow launching if prompted."
