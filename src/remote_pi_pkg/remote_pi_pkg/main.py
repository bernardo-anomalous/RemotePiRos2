import os
import subprocess
import signal
import logging
import atexit

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

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def cleanup_processes(processes):
    """Terminate external helper processes."""
    for name, proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                logger.warning(
                    "Process %s already terminated before SIGINT", name
                )
            except Exception:  # pragma: no cover - unexpected errors
                logger.exception("Failed to send SIGINT to %s", name)
    for name, proc in processes:
        if proc.poll() is None:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning(
                    "Process %s did not exit after SIGINT; killing", name
                )
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    logger.warning(
                        "Process %s disappeared before SIGKILL", name
                    )
                except Exception:  # pragma: no cover - unexpected errors
                    logger.exception("Failed to send SIGKILL to %s", name)
                proc.wait()

def ros_spin(node):
    rclpy.spin(node)

def main():

    rclpy.init()

    # Launch input nodes alongside the GUI in their own sessions so we can
    # cleanly terminate the entire process groups.
    joy_proc = subprocess.Popen(
        ['ros2', 'run', 'joy', 'joy_node'], start_new_session=True
    )
    mapper_proc = subprocess.Popen(
        ['ros2', 'run', 'remote_pi_pkg', 'gamepad_mapper'],
        start_new_session=True,
    )
    procs = [
        ('joy_node', joy_proc),
        ('gamepad_mapper', mapper_proc),
    ]
    atexit.register(cleanup_processes, procs)
    exit_code = 0
    ros_node = None
    try:
        # Start ROS node
        ros_node = ROSInterface()
        ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
        ros_thread.start()

        # Start Qt GUI
        app = QApplication(sys.argv)
        signal.signal(signal.SIGINT, lambda *args: app.quit())
        gui = AUVControlGUI(ros_node, joy_proc=joy_proc, mapper_proc=mapper_proc)
        gui.show()
        exit_code = app.exec_()
    finally:
        # Cleanup
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()

        cleanup_processes(procs)

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
