from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

from remote_pi_pkg.ros.interface import ROSInterface
from remote_pi_pkg.widgets.virtual_joystick import VirtualJoystickWidget
from remote_pi_pkg.widgets.heading_bar import HeadingBarWidget
from remote_pi_pkg.widgets.attitude_indicator import AttitudeIndicator
from remote_pi_pkg.widgets.control_status_field import ControlStatusField
from lifecycle_msgs.msg import Transition
from PyQt5.QtWidgets import QScrollArea, QSizePolicy
from PyQt5.QtGui import QTextOption
import time



import os

class AUVControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("NOSTROMO-AUV CONTROL")
        self.setGeometry(0, 0, 600, 1024)
        self.showFullScreen()

        # === GUI Styling and Background ===
        script_dir = os.path.dirname(os.path.realpath(__file__))
        bg_path = "/home/b/RemotePiRos2/assets/background.png"
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
                background-color: rgba(34, 34, 34, 180);
                border: 2px solid #00FFFF;
                margin-right: 4px;
            }}
            QTabBar::tab:selected {{
                background-color: rgba(51, 51, 51, 200);
                border-bottom: 4px solid #FF4500;
            }}
        """)

        # === BUILD THE GUI ===
        self.init_ui()

        # === NOW SAFELY START THE TIMER ===
        self.status_update_timer = QTimer()
        self.status_update_timer.timeout.connect(self.update_status)
        self.status_update_timer.start(50)

    def init_ui(self):
        
        

        # Add a timer to check lifecycle state every second
        self.lifecycle_update_timer = QTimer()
        self.lifecycle_update_timer.timeout.connect(self.update_lifecycle_buttons)
        self.lifecycle_update_timer.start(1000)  # Check every 1 second

        # === Tabs ===
        outer_layout = QVBoxLayout()
        tabs = QTabWidget()

        operation_tab = QWidget()
        settings_tab = QWidget()

        operation_layout = QVBoxLayout()
        main_layout = QHBoxLayout()

        operation_tab.setAttribute(Qt.WA_StyledBackground, True)
        operation_tab.setStyleSheet("background: transparent;")
        settings_tab.setAttribute(Qt.WA_StyledBackground, True)
        settings_tab.setStyleSheet("background: transparent;")

        tabs.addTab(operation_tab, "OPERATION")
        tabs.addTab(settings_tab, "SETTINGS")
        outer_layout.addWidget(tabs)
        self.setLayout(outer_layout)

        # === SETTINGS TAB ===
        settings_layout = QVBoxLayout()
        self.btn_configure = QPushButton("CONFIGURE DRIVER")
        self.btn_activate = QPushButton("ACTIVATE DRIVER")
        self.btn_deactivate = QPushButton("DEACTIVATE DRIVER")
        self.btn_cleanup = QPushButton("CLEANUP DRIVER")
        self.btn_shutdown = QPushButton("SHUTDOWN DRIVER")
        self.btn_error_recovery = QPushButton("ERROR RECOVERY")
        self.btn_quit = QPushButton("QUIT")
        self.btn_quit.setObjectName("quitButton")

        self.label_current_state = QLabel("CURRENT STATE: UNKNOWN")
        self.label_current_state.setStyleSheet("font-size: 18px; color: #00FFFF;")

        self.btn_quit.clicked.connect(self.quit_app)

        button_transition_pairs = [
            (self.btn_configure, Transition.TRANSITION_CONFIGURE),
            (self.btn_activate, Transition.TRANSITION_ACTIVATE),
            (self.btn_deactivate, Transition.TRANSITION_DEACTIVATE),
            (self.btn_cleanup, Transition.TRANSITION_CLEANUP),
            (self.btn_shutdown, Transition.TRANSITION_UNCONFIGURED_SHUTDOWN),
            (self.btn_error_recovery, Transition.TRANSITION_DESTROY)
        ]

        for btn, trans_id in button_transition_pairs:
         btn.clicked.connect(self.make_lifecycle_callback(trans_id))
         settings_layout.addWidget(btn)

        settings_layout.addWidget(self.label_current_state)
        settings_layout.addStretch(1)
        settings_tab.setLayout(settings_layout)
        settings_layout.addWidget(self.btn_quit)

        # === OPERATION TAB ===
        left_layout = QVBoxLayout()
        self.imu_health_label = QLabel("IMU HEALTH: UNKNOWN")
        self.imu_health_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
        left_layout.addWidget(self.imu_health_label)


        # NEW: Toggle button replacing old activate/deactivate buttons
        self.btn_pid_toggle = QPushButton("ROLL PID: INACTIVE")
        self.btn_pid_toggle.setCheckable(True)
        self.btn_pid_toggle.setStyleSheet("border: 2px solid #FF4500; color: #FF4500;")
        self.btn_pid_toggle.clicked.connect(self.toggle_pid)

        self.btn_canned = QPushButton("SEND CANNED MOVEMENT")
        self.btn_duration_increase = QPushButton("DURATION +")
        self.btn_duration_decrease = QPushButton("DURATION -")
        self.btn_toggle_sticky = QPushButton("STICKY MODE: OFF")
        self.btn_toggle_sticky.setObjectName("stickyOff")

        self.btn_toggle_sticky.clicked.connect(self.toggle_sticky_mode)
        self.btn_pid_toggle.clicked.connect(self.toggle_pid)
        self.btn_canned.clicked.connect(self.ros_node.publish_canned)
        self.btn_duration_increase.clicked.connect(self.increase_duration)
        self.btn_duration_decrease.clicked.connect(self.decrease_duration)

        # Add widgets to left layout
        left_layout.addWidget(self.btn_pid_toggle)
        left_layout.addWidget(self.btn_canned)
        left_layout.addWidget(self.btn_duration_increase)
        left_layout.addWidget(self.btn_duration_decrease)
        left_layout.addWidget(self.btn_toggle_sticky)

        # Control status area with scrolling
        self.control_status_field = ControlStatusField()
        self.control_status_field.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.control_status_field.setWordWrapMode(QTextOption.WordWrap)
        self.control_status_field.setLineWrapMode(QTextEdit.WidgetWidth)
        self.control_status_field.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.control_status_field.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self.control_status_field)
        left_layout.addWidget(QLabel("CONTROL STATUS"))
        left_layout.addWidget(scroll_area)
        left_layout.addStretch(1)

        main_layout.addLayout(left_layout, 1)

        # === RIGHT COLUMN ===
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

        # === STATUS UPDATE TIMER ===
        self.status_update_timer = QTimer()
        self.status_update_timer.timeout.connect(self.update_status)
        self.status_update_timer.start(50)
        self.update_lifecycle_buttons()  # Check the state immediately on startup


    def make_lifecycle_callback(self, transition_id):
        return lambda _: self.handle_lifecycle_button(transition_id)

        
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
        #self.ros_node.get_logger().info(f"DURATION FACTOR INCREASED: {self.ros_node.canned_duration_factor:.2f}")

    def decrease_duration(self):
        new_factor = self.ros_node.canned_duration_factor - self.ros_node.DURATION_STEP
        self.ros_node.canned_duration_factor = max(0.1, new_factor)
        #self.ros_node.get_logger().info(f"DURATION FACTOR DECREASED: {self.ros_node.canned_duration_factor:.2f}")

    def joystick_callback(self, norm_x, norm_y):
        max_angle = 90.0  # or change to 45.0 for tighter control

        # === Simple Debounce to prevent flooding ===
        if not hasattr(self, 'last_joystick_publish'):
            self.last_joystick_publish = time.time()
        now = time.time()
        if now - self.last_joystick_publish < 0.05:  # 50ms debounce = 20Hz max
            return
        self.last_joystick_publish = now

        # === Update target values ===
        self.ros_node.target_pitch = norm_y * max_angle
        self.ros_node.target_roll = norm_x * max_angle

        # === Publish commands ===
        self.ros_node.publish_pitch()
        self.ros_node.publish_roll()

    def update_status(self):
        now = time.time()
        if not hasattr(self, 'last_gui_update'):
            self.last_gui_update = now
        if now - self.last_gui_update < 0.5:  # Only update every 500ms
            return  # Skip the update if too soon
        self.last_gui_update = now

        def colorize(value):
            if isinstance(value, float) and value < 0:
                return f'<font color="#FF4500">{value:.1f}</font>'
            return f'{value:.1f}' if isinstance(value, float) else str(value)

        # === Prepare Control Status Field ===
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
            f"PITCH COMMAND: {colorize(pitch_cmd)}<br>"
            f"ROLL COMMAND: {colorize(roll_cmd)}<br>"
            f"CURRENT PITCH: {colorize(current_pitch)}<br>"
            f"CURRENT ROLL: {colorize(current_roll)}<br>"
            f"SERVO ANGLES:<br>{servo_text}<br>"
            f"CANNED DURATION FACTOR: {'<font color=\"#FF4500\">'+f'{self.ros_node.canned_duration_factor:.2f}'+'</font>' if self.ros_node.canned_duration_factor < 1.0 else f'{self.ros_node.canned_duration_factor:.2f}'}<br>"
            f"LAST COMMAND SENT: {self.ros_node.last_command}"
        )

        # === Prepare Heading, IMU, Euler, and Overall Status ===
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
            f"LIFECYCLE: {self.ros_node.current_lifecycle_state or 'UNAVAILABLE'}"

        )

        # === Selective Repainting (Only if Changed) ===
        if getattr(self, 'last_control_status', None) != control_status:
            self.control_status_field.setHtml(control_status)
            self.last_control_status = control_status

        if getattr(self, 'last_overall_status', None) != overall_status:
            self.status_display.setHtml(overall_status)
            self.last_overall_status = overall_status

        # === Always Update Heading Widget ===
        if self.ros_node.heading is not None:
            self.heading_hud.update_heading(self.ros_node.heading)

        # === Always Update Attitude Widget (Pose) ===
        if self.ros_node.euler is not None and len(self.ros_node.euler) >= 3:
            self.attitude_widget.update_attitude(self.ros_node.euler)
            self.attitude_widget.update()

        # === Update PID Toggle Button State ===
        self.update_pid_button_state()
        
        # === Update IMU Health Label ===
        imu_health = self.ros_node.imu_health_status

        if imu_health == "IMU OK":
            self.imu_health_label.setText("IMU HEALTH: OK")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #00FF00;")
        elif "UNSTABLE" in imu_health:
            self.imu_health_label.setText(f"IMU HEALTH: {imu_health}")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #FFA500;")  # Orange
        elif "RESTARTING" in imu_health:
            self.imu_health_label.setText(f"IMU HEALTH: {imu_health}")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #FF4500;")  # Red
        else:
            self.imu_health_label.setText("IMU HEALTH: UNKNOWN")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")


                        
                

            
    def update_lifecycle_buttons(self):
        current_state = self.ros_node.current_lifecycle_state


        # Avoid NoneType errors:
        if not current_state or current_state == "unavailable":
            # Gray out ALL buttons if no node detected
            for btn in [
                self.btn_configure, self.btn_activate, self.btn_deactivate,
                self.btn_cleanup, self.btn_shutdown, self.btn_error_recovery
            ]:
                btn.setEnabled(False)
                btn.setStyleSheet("border: 2px solid #555555; color: #555555;")
            self.label_current_state.setText("CURRENT STATE: NODE OFFLINE")
            return  # Exit early!

        # Example transition mapping
        transitions = [
            {'id': Transition.TRANSITION_CONFIGURE, 'start': 'unconfigured', 'label': 'CONFIGURE DRIVER'},
            {'id': Transition.TRANSITION_ACTIVATE, 'start': 'inactive', 'label': 'ACTIVATE DRIVER'},
            {'id': Transition.TRANSITION_DEACTIVATE, 'start': 'active', 'label': 'DEACTIVATE DRIVER'},
            {'id': Transition.TRANSITION_CLEANUP, 'start': 'inactive', 'label': 'CLEANUP DRIVER'},
            {'id': Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, 'start': 'unconfigured', 'label': 'SHUTDOWN (UNCONFIGURED)'},
            {'id': Transition.TRANSITION_INACTIVE_SHUTDOWN, 'start': 'inactive', 'label': 'SHUTDOWN (INACTIVE)'},
            {'id': Transition.TRANSITION_ACTIVE_SHUTDOWN, 'start': 'active', 'label': 'SHUTDOWN (ACTIVE)'},
            {'id': Transition.TRANSITION_DESTROY, 'start': 'errorprocessing', 'label': 'ERROR RECOVERY'}
        ]


        # Update button appearance based on state
        for transition in transitions:
            btn = None
            if transition['id'] == 1:
                btn = self.btn_configure
            elif transition['id'] == 3:
                btn = self.btn_activate
            elif transition['id'] == 4:
                btn = self.btn_deactivate
            elif transition['id'] == 5:
                btn = self.btn_cleanup

            if btn:
                if current_state and transition['start'].lower() == current_state.lower():
                    btn.setStyleSheet("border: 2px solid #00FF00; color: #00FF00;")
                    btn.setEnabled(True)
                else:
                    btn.setStyleSheet("border: 2px solid #555555; color: #555555;")
                    btn.setEnabled(False)

    def handle_lifecycle_button(self, transition_id):
        # Disable buttons temporarily to avoid multiple presses
        self.set_lifecycle_buttons_enabled(False)

        # Call the lifecycle service on the ros_node
        self.ros_node.call_lifecycle_service(transition_id)

        # Refresh the button states after some delay
        QTimer.singleShot(1000, self.update_lifecycle_buttons)
        QTimer.singleShot(1000, lambda: self.set_lifecycle_buttons_enabled(True))

    def set_lifecycle_buttons_enabled(self, enabled):
        for btn in [
            self.btn_configure,
            self.btn_activate,
            self.btn_deactivate,
            self.btn_cleanup,
            self.btn_shutdown,
            self.btn_error_recovery
        ]:
            btn.setEnabled(enabled)

            
        
    def toggle_pid(self):
        activate = self.btn_pid_toggle.isChecked()
        self.ros_node.set_pid(activate)
        if activate:
            self.btn_pid_toggle.setText("ROLL PID: ACTIVE")
            self.btn_pid_toggle.setStyleSheet("border: 2px solid #00FF00; color: #00FF00;")
        else:
            self.btn_pid_toggle.setText("ROLL PID: INACTIVE")
            self.btn_pid_toggle.setStyleSheet("border: 2px solid #FF4500; color: #FF4500;")
            
    def update_pid_button_state(self):
        if self.ros_node.roll_pid_enabled:
            self.btn_pid_toggle.setText("ROLL PID: ACTIVE")
            self.btn_pid_toggle.setChecked(True)
            self.btn_pid_toggle.setStyleSheet("border: 2px solid #00FF00; color: #00FF00;")
        else:
            self.btn_pid_toggle.setText("ROLL PID: INACTIVE")
            self.btn_pid_toggle.setChecked(False)
            self.btn_pid_toggle.setStyleSheet("border: 2px solid #FF4500; color: #FF4500;")


        

