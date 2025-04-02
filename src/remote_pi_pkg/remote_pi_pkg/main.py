import sys
import threading
import rclpy
from PyQt5.QtWidgets import QApplication

from remote_pi_pkg.ros.interface import ROSInterface
from remote_pi_pkg.auv_control_gui import AUVControlGUI

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()

    # Start ROS node
    ros_node = ROSInterface()
    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Start Qt GUI
    app = QApplication(sys.argv)
    gui = AUVControlGUI(ros_node)
    gui.show()
    sys.exit(app.exec_())

    # Cleanup (not reached due to sys.exit, but good form)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
