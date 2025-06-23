import os
import subprocess

# === Enable OpenGL acceleration ===
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # Try 'eglfs' if 'xcb' gives issues
os.environ['QT_OPENGL'] = 'egl'        # Options: 'egl', 'desktop', or 'angle'

import sys
import signal
import threading
import rclpy
from PyQt5.QtWidgets import QApplication
from remote_pi_pkg.ros.interface import ROSInterface
from remote_pi_pkg.auv_control_gui import AUVControlGUI

def ros_spin(node):
    rclpy.spin(node)

def main():

    rclpy.init()

    # Launch input nodes alongside the GUI
    joy_proc = subprocess.Popen(['ros2', 'run', 'joy', 'joy_node'])
    mapper_proc = subprocess.Popen(['ros2', 'run', 'remote_pi_pkg', 'gamepad_mapper'])
    exit_code = 0
    try:
        # Start ROS node
        ros_node = ROSInterface()
        ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
        ros_thread.start()

        # Start Qt GUI
        app = QApplication(sys.argv)
        signal.signal(signal.SIGINT, lambda *args: app.quit())
        gui = AUVControlGUI(ros_node)
        gui.show()
        exit_code = app.exec_()
    finally:
        # Cleanup
        ros_node.destroy_node()
        rclpy.shutdown()
        mapper_proc.terminate()
        joy_proc.terminate()
        mapper_proc.wait()
        joy_proc.wait()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
