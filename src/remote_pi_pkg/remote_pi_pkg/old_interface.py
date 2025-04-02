#!/usr/bin/env python3
import sys
import threading
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Float32MultiArray
from sensor_msgs.msg import Imu
from auv_custom_interfaces.msg import ServoMovementCommand
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QTextEdit, QComboBox, QGridLayout, QLabel)
from PyQt5.QtCore import Qt, QTimer, QPoint, QSize
from PyQt5.QtGui import QPainter, QPen, QColor, QFont

# ----------------- Virtual Joystick Widget -----------------
class VirtualJoystickWidget(QWidget):
    """
    A virtual joystick that displays a large circle with crosshairs and a draggable knob.
    The knob's position is normalized to (-1 to 1, -1 to 1) and reported via a callback.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.knob_radius = 20
        self.dragging = False
        self.knob_pos = self.rect().center()
        self.callback = None  # Callback: function(norm_x, norm_y)

    def sizeHint(self):
        return QSize(200, 200)

    def resizeEvent(self, event):
        self.knob_pos = self.rect().center()
        super().resizeEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        center = rect.center()
        radius = min(rect.width(), rect.height()) // 2 - 10

        pen = QPen(QColor("#00FF00"), 3)
        painter.setPen(pen)
        painter.drawEllipse(center, radius, radius)

        painter.drawLine(center.x() - radius, center.y(), center.x() + radius, center.y())
        painter.drawLine(center.x(), center.y() - radius, center.x(), center.y() + radius)

        painter.setBrush(QColor("#00FF00"))
        painter.drawEllipse(self.knob_pos, self.knob_radius, self.knob_radius)

    def mousePressEvent(self, event):
        if (event.pos() - self.knob_pos).manhattanLength() <= self.knob_radius:
            self.dragging = True

    def mouseMoveEvent(self, event):
        if self.dragging:
            center = self.rect().center()
            max_radius = min(self.rect().width(), self.rect().height()) // 2 - 10 - self.knob_radius
            offset = event.pos() - center
            distance = math.hypot(offset.x(), offset.y())
            if distance > max_radius and distance != 0:
                factor = max_radius / distance
                offset = offset * factor
            self.knob_pos = center + offset
            self.update()
            norm_x = offset.x() / max_radius
            norm_y = -offset.y() / max_radius  # Invert Y: up is positive
            if self.callback:
                self.callback(norm_x, norm_y)

    def mouseReleaseEvent(self, event):
        self.dragging = False
        self.knob_pos = self.rect().center()
        self.update()
        if self.callback:
            self.callback(0.0, 0.0)

# ----------------- Heading Bar HUD Widget -----------------
class HeadingBarWidget(QWidget):
    """
    A horizontal HUD-style heading bar.
    Draws a horizontal line with tick marks for cardinal directions and a vertical red marker.
    Also displays the exact heading value (in degrees) near the marker.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0  # degrees (0 = NORTH)
        self.setMinimumHeight(80)

    def update_heading(self, heading):
        self.heading = heading % 360
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        width = rect.width()
        height = rect.height()
        center_x = width // 2
        center_y = height // 2

        painter.fillRect(rect, QColor("#141414"))
        pen = QPen(QColor("#00FF00"), 2)
        painter.setPen(pen)
        painter.drawLine(0, center_y, width, center_y)

        tick_interval = 15
        pixels_per_degree = width / 90.0  # ±45° around center

        for deg in range(-45, 46, tick_interval):
            tick_x = center_x + int(deg * pixels_per_degree)
            painter.drawLine(tick_x, center_y - 10, tick_x, center_y + 10)
            label_deg = (self.heading + deg) % 360
            cardinal = ""
            if abs(label_deg - 0) < 7.5 or abs(label_deg - 360) < 7.5:
                cardinal = "N"
            elif abs(label_deg - 90) < 7.5:
                cardinal = "E"
            elif abs(label_deg - 180) < 7.5:
                cardinal = "S"
            elif abs(label_deg - 270) < 7.5:
                cardinal = "W"
            label = cardinal if cardinal else f"{int(label_deg)}°"
            painter.drawText(tick_x - 15, center_y + 30, label)

        marker_pen = QPen(QColor("#FF4500"), 3)
        painter.setPen(marker_pen)
        painter.drawLine(center_x, center_y - 20, center_x, center_y + 20)

        painter.setPen(QPen(QColor("#FF4500")))
        painter.setFont(QFont("Courier New", 14, QFont.Bold))
        heading_text = f"{self.heading:.1f}°"
        painter.drawText(center_x - 30, center_y - 25, heading_text)

# ----------------- Control Status Field Widget -----------------
class ControlStatusField(QTextEdit):
    """
    A read-only text area that displays control data.
    Shows: PITCH COMMAND, ROLL COMMAND, CURRENT PITCH, CURRENT ROLL, and SERVO ANGLES.
    Each item is on its own line.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setStyleSheet("QTextEdit { background-color: #222222; border: 2px solid #00FFFF; padding: 5px; }")
        self.setMinimumHeight(150)

# ----------------- ROS Interface Node -----------------
class ROSInterface(Node):
    def __init__(self):
        super().__init__('gui_ros_interface')
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.mode = "OPERATION"
        self.canned_duration_factor = 1.0
        self.DURATION_STEP = 0.05  # Adjustable increment

        # Sensor readouts
        self.current_servo_angles = []  # List of 6 servo angles
        self.imu_reading = None         # sensor_msgs/Imu
        self.heading = None             # Float32 heading (degrees)
        self.euler = None               # Euler angles: [roll, pitch, yaw]

        # Last command sent (for status display)
        self.last_command = "NONE"

        # Publishers
        self.target_roll_pub = self.create_publisher(Float32, 'target_roll', 10)
        self.target_pitch_pub = self.create_publisher(Float32, 'target_pitch', 10)
        self.canned_pub = self.create_publisher(ServoMovementCommand, 'servo_interpolation_commands', 10)
        self.pid_pub = self.create_publisher(Bool, 'wing_pid_active', 10)
        
        # Lifecycle service client
        self.lifecycle_client = self.create_client(ChangeState, 'servo_driver_node/change_state')
        if not self.lifecycle_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("LIFECYCLE SERVICE NOT AVAILABLE. CONTINUING WITHOUT IT.")

        # Subscribers
        self.create_subscription(Float32MultiArray, 'current_servo_angles', self.servo_angles_callback, 10)
        self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.create_subscription(Float32, 'imu/heading', self.heading_callback, 10)
        self.create_subscription(Float32MultiArray, 'imu/euler', self.euler_callback, 10)

    def publish_roll(self):
        msg = Float32()
        msg.data = self.target_roll
        self.target_roll_pub.publish(msg)
        self.get_logger().info(f"PUBLISHED ROLL: {self.target_roll}")

    def publish_pitch(self):
        msg = Float32()
        msg.data = self.target_pitch
        self.target_pitch_pub.publish(msg)
        self.get_logger().info(f"PUBLISHED PITCH: {self.target_pitch}")

    def publish_canned(self):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted_durations = [d * self.canned_duration_factor for d in base_durations]
        canned_commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [60.0, 50.0, 60.0, 110.0,
                              0.0, 50.0, 120.0, 110.0,
                              0.0, 90.0, 120.0, 90.0,
                              0.0, 130.0, 120.0, 30.0,
                              120.0, 110.0, 0.0, 50.0,
                              120.0, 90.0, 0.0, 90.0,
                              120.0, 50.0, 0.0, 110.0,
                              60.0, 50.0, 60.0, 110.0,
                              60.0, 90.0, 60.0, 90.0],
            'durations': adjusted_durations,
            'easing_algorithms': ['EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL', 
                                  'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.get_clock().now() + rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0
        }
        msg = ServoMovementCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.servo_numbers = canned_commands['servo_numbers']
        msg.target_angles = canned_commands['target_angles']
        msg.durations = canned_commands['durations']
        msg.easing_algorithms = canned_commands['easing_algorithms']
        msg.easing_in_factors = canned_commands['easing_in_factors']
        msg.easing_out_factors = canned_commands['easing_out_factors']
        msg.movement_type = canned_commands['movement_type']
        msg.deadline = canned_commands['deadline']
        msg.operational_mode = canned_commands['operational_mode']
        msg.priority = canned_commands['priority']
        
        self.canned_pub.publish(msg)
        self.last_command = f"CANNED MOVEMENT PUBLISHED @ {self.get_clock().now().to_msg()}"
        self.get_logger().info("PUBLISHED CANNED MOVEMENT COMMAND.")

    def set_pid(self, activate: bool):
        msg = Bool()
        msg.data = activate
        self.pid_pub.publish(msg)
        state = "ACTIVATED" if activate else "DEACTIVATED"
        self.get_logger().info(f"{state} ROLL PID.")

    def servo_angles_callback(self, msg: Float32MultiArray):
        self.current_servo_angles = list(msg.data)
        self.get_logger().info(f"SERVO ANGLES: {self.current_servo_angles}")

    def imu_callback(self, msg: Imu):
        self.imu_reading = msg
        self.get_logger().info("RECEIVED IMU DATA.")

    def heading_callback(self, msg: Float32):
        self.heading = msg.data
        self.get_logger().info(f"HEADING: {self.heading}")

    def euler_callback(self, msg: Float32MultiArray):
        try:
            # Ensure Euler values are floats
            self.euler = [float(x) for x in msg.data]
        except Exception:
            self.euler = None
        self.get_logger().info(f"EULER ANGLES: {self.euler}")

# ----------------- Attitude Indicator Widget -----------------
class AttitudeIndicator(QWidget):
    """
    A simple attitude indicator that draws an artificial horizon.
    Uses Euler angles [roll, pitch, yaw] to shift a horizon line.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update_attitude(self, euler):
        if euler and len(euler) >= 3:
            try:
                self.roll = float(euler[0])
                self.pitch = float(euler[1])
                self.yaw = float(euler[2])
            except Exception:
                self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        painter.fillRect(rect, QColor("#141414"))
        center = rect.center()
        radius = min(rect.width(), rect.height()) // 2 - 10

        # Calculate horizon position: pitch shifts vertically, roll rotates the horizon.
        pitch_offset = self.pitch * 2  # scale factor
        line_length = rect.width() * 1.5
        start = QPoint(center.x() - line_length // 2, center.y() + pitch_offset)
        end = QPoint(center.x() + line_length // 2, center.y() + pitch_offset)

        painter.translate(center)
        painter.rotate(-self.roll)
        painter.translate(-center)
        pen = QPen(QColor("#00FF00"), 3)
        painter.setPen(pen)
        painter.drawLine(start, end)
        painter.drawEllipse(center, 5, 5)
        painter.resetTransform()

# ----------------- Heading HUD (Horizontal Bar) Widget -----------------
class HeadingBarHUD(QWidget):
    """
    A horizontal heading bar HUD.
    Draws a horizontal line with tick marks and a vertical red marker.
    Displays the exact heading value near the marker.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0
        self.setMinimumHeight(80)

    def update_heading(self, heading):
        self.heading = heading % 360
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        width = rect.width()
        height = rect.height()
        center_x = width // 2
        center_y = height // 2

        painter.fillRect(rect, QColor("#141414"))
        pen = QPen(QColor("#00FF00"), 2)
        painter.setPen(pen)
        painter.drawLine(0, center_y, width, center_y)

        tick_interval = 15
        pixels_per_degree = width / 90.0  # ±45° around center

        for deg in range(-45, 46, tick_interval):
            tick_x = center_x + int(deg * pixels_per_degree)
            painter.drawLine(tick_x, center_y - 10, tick_x, center_y + 10)
            label_deg = (self.heading + deg) % 360
            cardinal = ""
            if abs(label_deg - 0) < 7.5 or abs(label_deg - 360) < 7.5:
                cardinal = "N"
            elif abs(label_deg - 90) < 7.5:
                cardinal = "E"
            elif abs(label_deg - 180) < 7.5:
                cardinal = "S"
            elif abs(label_deg - 270) < 7.5:
                cardinal = "W"
            label = cardinal if cardinal else f"{int(label_deg)}°"
            painter.drawText(tick_x - 15, center_y + 30, label)

        marker_pen = QPen(QColor("#FF4500"), 3)
        painter.setPen(marker_pen)
        painter.drawLine(center_x, center_y - 20, center_x, center_y + 20)

        painter.setPen(QPen(QColor("#FF4500")))
        painter.setFont(QFont("Courier New", 14, QFont.Bold))
        heading_text = f"{self.heading:.1f}°"
        painter.drawText(center_x - 30, center_y - 25, heading_text)

# ----------------- PyQt5 GUI -----------------
class AUVControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        self.status_update_timer = QTimer()
        self.status_update_timer.timeout.connect(self.update_status)
        self.status_update_timer.start(1000)

    def init_ui(self):
        # Set window for portrait mode with extra padding (650x1024)
        self.setWindowTitle("NOSTROMO-INSPIRED AUV CONTROL")
        self.setGeometry(0, 0, 650, 1024)
        self.showFullScreen()

        self.setStyleSheet("""
            QWidget {
                background-color: #141414;
                color: #00FF00;
                font-family: "COURIER NEW", monospace;
                font-size: 16px;
            }
            QPushButton {
                background-color: #222222;
                border: 2px solid #00FFFF;
                padding: 10px;
            }
            QPushButton#deactivateButton {
                border: 2px solid #FF4500;
                color: #FF4500;
            }
            QPushButton:hover {
                background-color: #333333;
            }
            QTextEdit {
                background-color: #222222;
                border: 2px solid #00FFFF;
                padding: 5px;
            }
            QComboBox {
                background-color: #222222;
                border: 2px solid #00FFFF;
                padding: 5px;
            }
        """)

        outer_layout = QVBoxLayout()

        # Top row: Main layout with two columns
        main_layout = QHBoxLayout()

        # Left column: Control buttons and status field.
        left_layout = QVBoxLayout()
        self.btn_activate_pid = QPushButton("ACTIVATE ROLL PID")
        self.btn_deactivate_pid = QPushButton("DEACTIVATE ROLL PID")
        self.btn_deactivate_pid.setObjectName("deactivateButton")
        self.btn_canned = QPushButton("SEND CANNED MOVEMENT")
        left_layout.addWidget(self.btn_activate_pid)
        left_layout.addWidget(self.btn_deactivate_pid)
        left_layout.addWidget(self.btn_canned)
        self.btn_duration_increase = QPushButton("DURATION +")
        self.btn_duration_decrease = QPushButton("DURATION -")
        left_layout.addWidget(self.btn_duration_increase)
        left_layout.addWidget(self.btn_duration_decrease)
        self.control_status_field = ControlStatusField()
        left_layout.addWidget(QLabel("CONTROL STATUS"))
        left_layout.addWidget(self.control_status_field)
        left_layout.addStretch(1)
        main_layout.addLayout(left_layout, 1)

        # Right column: Virtual Joystick and Heading HUD.
        right_layout = QVBoxLayout()
        self.virtual_joystick = VirtualJoystickWidget()
        self.virtual_joystick.callback = self.joystick_callback
        right_layout.addWidget(QLabel("VIRTUAL JOYSTICK"))
        right_layout.addWidget(self.virtual_joystick)
        self.heading_hud = HeadingBarWidget()
        right_layout.addWidget(QLabel("HEADING HUD"))
        right_layout.addWidget(self.heading_hud)
        right_layout.addStretch(1)
        main_layout.addLayout(right_layout, 1)

        outer_layout.addLayout(main_layout)

        # Bottom row: Overall System Status display area.
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        outer_layout.addWidget(QLabel("SYSTEM STATUS"))
        outer_layout.addWidget(self.status_display)

        self.setLayout(outer_layout)

        # Connect signals to actions.
        self.btn_activate_pid.clicked.connect(lambda: self.ros_node.set_pid(True))
        self.btn_deactivate_pid.clicked.connect(lambda: self.ros_node.set_pid(False))
        self.btn_canned.clicked.connect(self.ros_node.publish_canned)
        self.btn_duration_increase.clicked.connect(self.increase_duration)
        self.btn_duration_decrease.clicked.connect(self.decrease_duration)

    def increase_duration(self):
        self.ros_node.canned_duration_factor += self.ros_node.DURATION_STEP
        self.ros_node.get_logger().info(f"DURATION FACTOR INCREASED: {self.ros_node.canned_duration_factor:.2f}")

    def decrease_duration(self):
        new_factor = self.ros_node.canned_duration_factor - self.ros_node.DURATION_STEP
        self.ros_node.canned_duration_factor = max(0.1, new_factor)
        self.ros_node.get_logger().info(f"DURATION FACTOR DECREASED: {self.ros_node.canned_duration_factor:.2f}")

    def joystick_callback(self, norm_x, norm_y):
        # Map joystick normalized values to pitch and roll adjustments.
        delta_pitch = norm_y * 5  # sensitivity factor
        delta_roll = norm_x * 5
        self.ros_node.target_pitch = max(-100, min(100, self.ros_node.target_pitch + delta_pitch))
        self.ros_node.target_roll = max(-100, min(100, self.ros_node.target_roll + delta_roll))
        self.ros_node.publish_pitch()
        self.ros_node.publish_roll()

    def update_status(self):
        # Build Control Status Field.
        try:
            pitch_cmd = self.ros_node.target_pitch
            roll_cmd = self.ros_node.target_roll
        except Exception:
            pitch_cmd = roll_cmd = 0.0
        current_pitch = "N/A"
        current_roll = "N/A"
        if self.ros_node.euler and len(self.ros_node.euler) >= 2:
            try:
                current_roll = float(self.ros_node.euler[0])
                current_pitch = float(self.ros_node.euler[1])
            except Exception:
                current_roll, current_pitch = "N/A", "N/A"
        servo_text = "N/A"
        if self.ros_node.current_servo_angles:
            servo_text = "<br>".join(f"SERVO {i+1}: {angle:.1f}" for i, angle in enumerate(self.ros_node.current_servo_angles))
        control_status = (
            f"PITCH COMMAND: {pitch_cmd:.1f}<br>"
            f"ROLL COMMAND: {roll_cmd:.1f}<br>"
            f"CURRENT PITCH: {'<font color=\"#FF4500\">'+f'{current_pitch:.1f}'+'</font>' if isinstance(current_pitch, float) and current_pitch < 0 else (f'{current_pitch:.1f}' if isinstance(current_pitch, float) else current_pitch)}<br>"
            f"CURRENT ROLL: {'<font color=\"#FF4500\">'+f'{current_roll:.1f}'+'</font>' if isinstance(current_roll, float) and current_roll < 0 else (f'{current_roll:.1f}' if isinstance(current_roll, float) else current_roll)}<br>"
            f"SERVO ANGLES:<br>{servo_text}<br>"
            f"CANNED DURATION FACTOR: {self.ros_node.canned_duration_factor:.2f}<br>"
            f"LAST COMMAND SENT: {self.ros_node.last_command}"
        )
        self.control_status_field.setHtml(control_status)

        # Build Overall System Status.
        heading_text = "N/A"
        if self.ros_node.heading is not None:
            heading = self.ros_node.heading % 360
            if heading < 22.5 or heading >= 337.5:
                card = "N"
            elif heading < 67.5:
                card = "NE"
            elif heading < 112.5:
                card = "E"
            elif heading < 157.5:
                card = "SE"
            elif heading < 202.5:
                card = "S"
            elif heading < 247.5:
                card = "SW"
            elif heading < 292.5:
                card = "W"
            else:
                card = "NW"
            heading_text = f"{heading:.1f}° ({card})"
        imu_text = "N/A"
        if self.ros_node.imu_reading is not None:
            imu = self.ros_node.imu_reading.orientation
            imu_text = f"IMU ORIENTATION: X={imu.x:.2f}, Y={imu.y:.2f}, Z={imu.z:.2f}, W={imu.w:.2f}"
        euler_text = "N/A"
        if self.ros_node.euler and len(self.ros_node.euler) >= 3:
            try:
                euler_text = f"CURRENT POSE: ROLL={self.ros_node.euler[0]:.1f}, PITCH={self.ros_node.euler[1]:.1f}, YAW={self.ros_node.euler[2]:.1f}"
            except Exception:
                euler_text = "N/A"
        overall_status = (
            f"MODE: {self.ros_node.mode}<br>"
            f"HEADING: {heading_text}<br>"
            f"{imu_text}<br>"
            f"{euler_text}<br>"
            "PID STATE: (SEE ROS LOGS)<br>"
            "LIFECYCLE: ACTIVE"
        )
        self.status_display.setHtml(overall_status)
        if self.ros_node.heading is not None:
            self.heading_hud.update_heading(self.ros_node.heading)

# ----------------- ROS Spinning in Background -----------------
def ros_spin(ros_node):
    rclpy.spin(ros_node)

# ----------------- Main Function -----------------
def main():
    rclpy.init()
    ros_node = ROSInterface()

    # Run the ROS node in a background thread
    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = AUVControlGUI(ros_node)
    gui.show()
    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
