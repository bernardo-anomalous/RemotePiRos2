from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

from remote_pi_pkg.ros.interface import ROSInterface

from remote_pi_pkg.widgets.virtual_joystick import VirtualJoystickWidget
from remote_pi_pkg.widgets.heading_bar import HeadingBarWidget
from remote_pi_pkg.widgets.attitude_indicator import AttitudeIndicator
from remote_pi_pkg.widgets.control_status_field import ControlStatusField
from PyQt5.QtWidgets import QTabWidget

import os




from remote_pi_pkg.ros.interface import ROSInterface


class AUVControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        self.status_update_timer = QTimer()
        self.status_update_timer.timeout.connect(self.update_status)
        self.status_update_timer.start(250)

    def init_ui(self):
        self.setWindowTitle("NOSTROMO-AUV CONTROL")
        self.setGeometry(0, 0, 600, 1024)
        self.showFullScreen()
        script_dir = os.path.dirname(os.path.realpath(__file__))
        bg_path = "/home/b/RemotePiRos2/assets/background.png"

        print(f"Resolved background path: {bg_path}")


        self.setStyleSheet(f"""
            QWidget {{
                background-image: url("{bg_path}");
                background-repeat: no-repeat;
                background-position: center;
                background-attachment: fixed;
                background-color: transparent;
                color: #00FF00;
                font-family: "COURIER NEW", monospace;
                font-size: 16px;
            }}
            QPushButton {{
                background-color: rgba(34, 34, 34, 220);
                border: 2px solid #00FFFF;
                padding: 10px;
            }}
            QPushButton#deactivateButton,
            QPushButton#quitButton {{
                border: 2px solid #FF4500;
                color: #FF4500;
            }}
            QPushButton:hover {{
                background-color: rgba(51, 51, 51, 220);
            }}
            QTextEdit {{
                background-color: rgba(34, 34, 34, 200);
                border: 2px solid #00FFFF;
                padding: 5px;
            }}
            QTabWidget::pane {{
                background: transparent;
            }}
            QTabBar::tab {{
                height: 30px;
                width: 260px;
                padding: 10px;
                font-size: 18px;
                background-color: rgba(34, 34, 34, 180);  /* Transparent background */
                border: 2px solid #00FFFF;
                margin-right: 4px;
            }}
            QTabBar::tab:selected {{
                background-color: rgba(51, 51, 51, 200);
                border-bottom: 4px solid #FF4500;
            }}
        """)


        # Root layout for tabs
        outer_layout = QVBoxLayout()
        tabs = QTabWidget()
        #tabs.setStyleSheet("background: transparent;")
        


        # =============== OPERATION TAB ===============
        operation_tab = QWidget()
        operation_layout = QVBoxLayout()
        main_layout = QHBoxLayout()
        operation_tab.setAttribute(Qt.WA_StyledBackground, True)
        operation_tab.setStyleSheet("background: transparent;")
        
                # =============== SETTINGS TAB ===============
        settings_tab = QWidget()
        settings_layout = QVBoxLayout()
        
        settings_tab.setAttribute(Qt.WA_StyledBackground, True)
        settings_tab.setStyleSheet("background: transparent;")

                # =============== ADD TABS ===============
        tabs.addTab(operation_tab, "OPERATION")
        tabs.addTab(settings_tab, "SETTINGS")

        outer_layout.addWidget(tabs)
        self.setLayout(outer_layout)
        QTimer.singleShot(0, self.showFullScreen)


        self.btn_configure = QPushButton("CONFIGURE DRIVER")
        self.btn_activate = QPushButton("ACTIVATE DRIVER")
        self.btn_deactivate = QPushButton("DEACTIVATE DRIVER")
        self.btn_cleanup = QPushButton("CLEANUP DRIVER")
        self.btn_quit = QPushButton("QUIT")
        self.btn_quit.setObjectName("quitButton")

        self.btn_quit.clicked.connect(self.quit_app)
        self.btn_configure.clicked.connect(lambda: self.ros_node.call_lifecycle_service(1))
        self.btn_activate.clicked.connect(lambda: self.ros_node.call_lifecycle_service(3))
        self.btn_deactivate.clicked.connect(lambda: self.ros_node.call_lifecycle_service(4))
        self.btn_cleanup.clicked.connect(lambda: self.ros_node.call_lifecycle_service(5))

        for btn in [self.btn_configure, self.btn_activate, self.btn_deactivate, self.btn_cleanup, self.btn_quit]:
            settings_layout.addWidget(btn)
        settings_layout.addStretch(1)
        settings_tab.setLayout(settings_layout)


        # --- LEFT COLUMN ---
        left_layout = QVBoxLayout()
        self.btn_activate_pid = QPushButton("ACTIVATE ROLL PID")
        self.btn_deactivate_pid = QPushButton("DEACTIVATE ROLL PID")
        self.btn_deactivate_pid.setObjectName("deactivateButton")
        self.btn_canned = QPushButton("SEND CANNED MOVEMENT")
        self.btn_duration_increase = QPushButton("DURATION +")
        self.btn_duration_decrease = QPushButton("DURATION -")
        self.btn_toggle_sticky = QPushButton("STICKY MODE: OFF")
        self.btn_toggle_sticky.setObjectName("stickyOff")

        self.btn_toggle_sticky.clicked.connect(self.toggle_sticky_mode)
        self.btn_activate_pid.clicked.connect(lambda: self.ros_node.set_pid(True))
        self.btn_deactivate_pid.clicked.connect(lambda: self.ros_node.set_pid(False))
        self.btn_canned.clicked.connect(self.ros_node.publish_canned)
        self.btn_duration_increase.clicked.connect(self.increase_duration)
        self.btn_duration_decrease.clicked.connect(self.decrease_duration)

        for btn in [self.btn_activate_pid, self.btn_deactivate_pid, self.btn_canned,
                    self.btn_duration_increase, self.btn_duration_decrease, self.btn_toggle_sticky]:
            left_layout.addWidget(btn)

        self.control_status_field = ControlStatusField()
        left_layout.addWidget(QLabel("CONTROL STATUS"))
        left_layout.addWidget(self.control_status_field)
        left_layout.addStretch(1)
        main_layout.addLayout(left_layout, 1)

        # --- RIGHT COLUMN ---
        right_layout = QVBoxLayout()
        self.heading_hud = HeadingBarWidget()
        self.attitude_widget = AttitudeIndicator()
        self.virtual_joystick = VirtualJoystickWidget()
        self.virtual_joystick.callback = self.joystick_callback

        right_layout.addWidget(QLabel("HEADING HUD"))
        right_layout.addWidget(self.heading_hud)
        right_layout.addWidget(QLabel("ATTITUDE INDICATOR"))
        right_layout.addWidget(self.attitude_widget)
        right_layout.addWidget(QLabel("VIRTUAL JOYSTICK"))
        right_layout.addWidget(self.virtual_joystick)
        right_layout.addStretch(1)
        main_layout.addLayout(right_layout, 1)

        operation_layout.addLayout(main_layout)
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        operation_layout.addWidget(QLabel("SYSTEM STATUS"))
        operation_layout.addWidget(self.status_display)
        operation_tab.setLayout(operation_layout)




        
    def toggle_sticky_mode(self):
        self.virtual_joystick.sticky_mode = not self.virtual_joystick.sticky_mode

        if self.virtual_joystick.sticky_mode:
            self.btn_toggle_sticky.setText("STICKY MODE: ON")
            self.btn_toggle_sticky.setStyleSheet("border: 2px solid #FF4500; color: #FF4500;")
        else:
            self.btn_toggle_sticky.setText("STICKY MODE: OFF")
            self.btn_toggle_sticky.setStyleSheet("border: 2px solid #00FF00; color: #00FF00;")

        
        
    def quit_app(self):
        QApplication.quit()

    def increase_duration(self):
        self.ros_node.canned_duration_factor += self.ros_node.DURATION_STEP
        self.ros_node.get_logger().info(f"DURATION FACTOR INCREASED: {self.ros_node.canned_duration_factor:.2f}")

    def decrease_duration(self):
        new_factor = self.ros_node.canned_duration_factor - self.ros_node.DURATION_STEP
        self.ros_node.canned_duration_factor = max(0.1, new_factor)
        self.ros_node.get_logger().info(f"DURATION FACTOR DECREASED: {self.ros_node.canned_duration_factor:.2f}")

    def joystick_callback(self, norm_x, norm_y):
        # Set target values directly instead of incrementing
        max_angle = 90.0  # or change to 45.0 for tighter control

        self.ros_node.target_pitch = norm_y * max_angle
        self.ros_node.target_roll = norm_x * max_angle

        self.ros_node.publish_pitch()
        self.ros_node.publish_roll()

    def update_status(self):
        
        def colorize(value):
            if isinstance(value, float) and value < 0:
                return f'<font color="#FF4500">{value:.1f}</font>'
            return f'{value:.1f}' if isinstance(value, float) else str(value)

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
            servo_text = "<br>".join(
                f"SERVO {i + 1}: {angle:.1f}" for i, angle in enumerate(self.ros_node.current_servo_angles)
            )

        current_pitch_text = colorize(current_pitch)
        current_roll_text = colorize(current_roll)

        factor_text = f"{self.ros_node.canned_duration_factor:.2f}"
        if self.ros_node.canned_duration_factor < 1.0:
            factor_text = f"<font color=\"#FF4500\">{factor_text}</font>"

        control_status = (
            f"PITCH COMMAND: {colorize(pitch_cmd)}<br>"
            f"ROLL COMMAND: {colorize(roll_cmd)}<br>"
            f"CURRENT PITCH: {current_pitch_text}<br>"
            f"CURRENT ROLL: {current_roll_text}<br>"
            f"SERVO ANGLES:<br>{servo_text}<br>"
            f"CANNED DURATION FACTOR: {factor_text}<br>"
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