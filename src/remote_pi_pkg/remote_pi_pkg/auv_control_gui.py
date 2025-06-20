from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel, QTabWidget, QDoubleSpinBox,
    QScrollArea, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
import re
from PyQt5.QtGui import QFont

from remote_pi_pkg.ros.interface import ROSInterface
from remote_pi_pkg.widgets.virtual_joystick import VirtualJoystickWidget
from remote_pi_pkg.widgets.heading_bar import HeadingBarWidget
from remote_pi_pkg.widgets.attitude_indicator import AttitudeIndicator
from remote_pi_pkg.widgets.control_status_field import ControlStatusField
from lifecycle_msgs.msg import Transition
from PyQt5.QtGui import QTextOption
import time
import os

class AUVControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("P.A.T.C.H-AUV CONTROL")
        self.setFixedSize(600, 980)  # Completely locks resizing


        #self.showFullScreen()
        screen_geometry = QApplication.desktop().screenGeometry()
        self.setGeometry(screen_geometry)                  # Explicitly set the window size to match the screen
        #self.setFixedSize(screen_geometry.width(), screen_geometry.height())  # Lock it down fully


        # === GUI Styling and Background ===
        script_dir = os.path.dirname(os.path.realpath(__file__))
        bg_path = "/home/b/RemotePiRos2/assets/background.png"
        self.setStyleSheet(f"""
            /* === TRANSPARENCY FIX FOR QTextEdit === */
            QTextEdit, QTextEdit QAbstractScrollArea, QTextEdit::viewport {{
                background-color: transparent;
            }}

            /* === GLOBAL WIDGET STYLING === */
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

            QTabWidget::pane {{
                background: transparent;
            }}

            QTabBar::tab {{
                height: 30px;
                width: 140px; /* Reduced from 180px to fit more tabs */
                padding: 10px;
                font-size: 18px;
                background-color: rgba(34, 34, 34, 180);
                border: 2px solid #00FFFF;
                margin-right: 4px;
            }}

            /* Bigger scroll arrows for easier navigation */
            QTabBar QToolButton {{
                width: 30px;
                height: 30px;
            }}

            QTabBar::left-arrow, QTabBar::right-arrow {{
                width: 30px;
                height: 30px;
            }}

            QTabBar::tab:selected {{
                background-color: rgba(51, 51, 51, 200);
                border-bottom: 4px solid #FF4500;
            }}

            QScrollBar:vertical {{
                width: 30px;
                background: rgba(34, 34, 34, 220);
            }}

            QScrollBar::handle:vertical {{
                background: #00FF00;
                min-height: 40px;
            }}

            QScrollBar::add-line:vertical, 
            QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
        """)


        # === BUILD THE GUI ===
        self.init_ui()

        # Allow ROS node to trigger UI updates when lifecycle state changes
        self.ros_node.lifecycle_update_callback = self.update_lifecycle_buttons



        # === NOW SAFELY START THE TIMER ===
        self.status_update_timer = QTimer()
        self.status_update_timer.timeout.connect(self.update_status)
        self.status_update_timer.start(500)
        # Visual update timer (runs at 60Hz)
        self.visual_update_timer = QTimer()
        self.visual_update_timer.timeout.connect(self.update_visuals)
        self.visual_update_timer.start(16)  # About 60 FPS
        
    def update_visuals(self):
        # Send target values to widgets every frame
        if self.ros_node.euler is not None and len(self.ros_node.euler) >= 3:
            self.attitude_widget.update_attitude_target(self.ros_node.euler)
            if hasattr(self, 'nav_attitude_widget'):
                self.nav_attitude_widget.update_attitude_target(self.ros_node.euler)
            
            # ✅ ADD THIS to fix your depth gauge:
        if hasattr(self.ros_node, 'depth'):
            self.attitude_widget.set_depth(self.ros_node.depth)
            if hasattr(self, 'nav_attitude_widget'):
                self.nav_attitude_widget.set_depth(self.ros_node.depth)

        if self.ros_node.heading is not None:
            self.heading_hud.update_heading_target(self.ros_node.heading)
            if hasattr(self, 'nav_heading'):
                self.nav_heading.update_heading_target(self.ros_node.heading)
            self.attitude_widget.update_heading_target(self.ros_node.heading)  # <<< THIS WAS MISSING
            if hasattr(self, 'nav_heading'):
                self.nav_heading.update_heading_target(self.ros_node.heading)
            if hasattr(self, 'nav_attitude_widget'):
                self.nav_attitude_widget.update_heading_target(self.ros_node.heading)

        # Force repaint (rely on the widget's smoothing)
        self.attitude_widget.update()
        self.heading_hud.update()
        if hasattr(self, 'nav_attitude_widget'):
            self.nav_attitude_widget.update()
        if hasattr(self, 'nav_heading'):
            self.nav_heading.update()



    def init_ui(self):
        # === Lifecycle Timer ===
        self.lifecycle_update_timer = QTimer()
        self.lifecycle_update_timer.timeout.connect(self.update_lifecycle_buttons)
        self.lifecycle_update_timer.start(1000)

        outer_layout = QVBoxLayout()
        tabs = QTabWidget()

        operation_tab = QWidget()
        navigation_tab = QWidget()
        settings_tab = QWidget()
        manual_tab = QWidget()

        operation_layout = QVBoxLayout()

        # === Transparent backgrounds for main tabs ===
        operation_tab.setAttribute(Qt.WA_StyledBackground, True)
        operation_tab.setStyleSheet("background: transparent;")
        navigation_tab.setAttribute(Qt.WA_StyledBackground, True)
        navigation_tab.setStyleSheet("background: transparent;")
        settings_tab.setAttribute(Qt.WA_StyledBackground, True)
        settings_tab.setStyleSheet("background: transparent;")
        manual_tab.setAttribute(Qt.WA_StyledBackground, True)
        manual_tab.setStyleSheet("background: transparent;")

        tabs.addTab(operation_tab, "OPERATION")
        tabs.addTab(navigation_tab, "NAVIGATION")
        tabs.addTab(manual_tab, "MANUAL INPUT")
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
        settings_layout.addWidget(self.btn_quit)
        settings_tab.setLayout(settings_layout)

        # === MANUAL INPUT TAB ===
        manual_layout = QVBoxLayout()

        # --- Step Duration Controls ---
        duration_row = QHBoxLayout()
        duration_row.addWidget(QLabel("STEP DURATION"))

        self.manual_duration_spin = QDoubleSpinBox()
        self.manual_duration_spin.setRange(0.1, 10.0)
        self.manual_duration_spin.setSingleStep(0.1)
        self.manual_duration_spin.setValue(1.0)
        self.manual_duration_spin.hide()  # Hide the small spin box arrows

        self.manual_duration_label = QLabel(f"{self.manual_duration_spin.value():.1f}s")
        self.manual_duration_label.setAlignment(Qt.AlignCenter)
        self.manual_duration_label.setStyleSheet("font-size: 24px;")

        self.btn_step_duration_decrease = QPushButton("-")
        self.btn_step_duration_increase = QPushButton("+")

        self.btn_step_duration_decrease.clicked.connect(self.manual_duration_spin.stepDown)
        self.btn_step_duration_increase.clicked.connect(self.manual_duration_spin.stepUp)
        self.manual_duration_spin.valueChanged.connect(self.update_manual_duration_label)

        duration_row.addWidget(self.btn_step_duration_decrease)
        duration_row.addWidget(self.manual_duration_label)
        duration_row.addWidget(self.btn_step_duration_increase)

        manual_layout.addLayout(duration_row)
        manual_layout.addWidget(self.manual_duration_spin)

        # Manual step feedback toggle
        self.manual_feedback_enabled = True
        self.last_manual_button = None
        self.btn_manual_feedback_toggle = QPushButton("STEP FEEDBACK: ON")
        self.btn_manual_feedback_toggle.setCheckable(True)
        self.btn_manual_feedback_toggle.setChecked(True)
        self.btn_manual_feedback_toggle.setStyleSheet(
            "border: 2px solid #00FF00; color: #00FF00;"
        )
        self.btn_manual_feedback_toggle.clicked.connect(
            self.toggle_manual_feedback
        )
        manual_layout.addWidget(self.btn_manual_feedback_toggle)

        # Build manual step buttons dynamically from available canned movements
        self.manual_step_buttons = []
        canned_re = re.compile(r'^canned_(\d+)(?:_(.*))?$')
        method_names = [m for m in dir(self.ros_node.canned_movements) if canned_re.match(m)]
        method_names.sort(key=lambda n: int(canned_re.match(n).group(1)))
        for name in method_names:
            match = canned_re.match(name)
            suffix = match.group(2)
            if suffix:
                label = suffix.replace('_', ' ').upper()
            else:
                label = match.group(1)
            btn = QPushButton(label)
            btn.setFixedHeight(35)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            method = getattr(self.ros_node.canned_movements, name)
            btn.clicked.connect(
                self.make_manual_step_handler(btn, method)
            )
            manual_layout.addWidget(btn)
            self.manual_step_buttons.append(btn)

        manual_layout.addStretch(1)
        manual_tab.setLayout(manual_layout)

        # === NAVIGATION TAB ===
        navigation_layout = QVBoxLayout()

        self.nav_heading = HeadingBarWidget()
        self.nav_heading.setMaximumHeight(80)
        navigation_layout.addWidget(self.nav_heading, alignment=Qt.AlignTop)

        self.nav_attitude_widget = AttitudeIndicator()
        navigation_layout.addWidget(self.nav_attitude_widget)
        self.ros_node.nav_attitude_widget = self.nav_attitude_widget

        nav_main_layout = QHBoxLayout()

        # Left side: canned movement buttons
        nav_left_layout = QVBoxLayout()
        self.navigation_step_buttons = []
        for name in method_names:
            match = canned_re.match(name)
            suffix = match.group(2)
            if suffix:
                label = suffix.replace('_', ' ').upper()
            else:
                label = match.group(1)
            btn = QPushButton(label)
            btn.setFixedHeight(35)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            method = getattr(self.ros_node.canned_movements, name)
            btn.clicked.connect(
                self.make_nav_step_handler(method)
            )
            nav_left_layout.addWidget(btn)
            self.navigation_step_buttons.append(btn)

        nav_left_layout.addStretch(1)
        nav_main_layout.addLayout(nav_left_layout, 1)

        # Right side: joystick
        nav_right_layout = QVBoxLayout()
        self.navigation_joystick = VirtualJoystickWidget()
        self.navigation_joystick.setFixedSize(250, 250)
        self.navigation_joystick.callback = self.nav_joystick_callback
        nav_right_layout.addWidget(self.navigation_joystick)

        # --- Navigation Step Duration Controls ---
        nav_duration_row = QHBoxLayout()
        nav_duration_row.addWidget(QLabel("STEP DURATION"))

        self.navigation_duration_spin = QDoubleSpinBox()
        self.navigation_duration_spin.setRange(0.1, 10.0)
        self.navigation_duration_spin.setSingleStep(0.1)
        self.navigation_duration_spin.setValue(1.0)
        self.navigation_duration_spin.hide()

        self.navigation_duration_label = QLabel(f"{self.navigation_duration_spin.value():.1f}s")
        self.navigation_duration_label.setAlignment(Qt.AlignCenter)
        self.navigation_duration_label.setStyleSheet("font-size: 24px;")

        self.btn_nav_duration_decrease = QPushButton("-")
        self.btn_nav_duration_increase = QPushButton("+")

        self.btn_nav_duration_decrease.clicked.connect(self.navigation_duration_spin.stepDown)
        self.btn_nav_duration_increase.clicked.connect(self.navigation_duration_spin.stepUp)
        self.navigation_duration_spin.valueChanged.connect(self.update_nav_duration_label)

        nav_duration_row.addWidget(self.btn_nav_duration_decrease)
        nav_duration_row.addWidget(self.navigation_duration_label)
        nav_duration_row.addWidget(self.btn_nav_duration_increase)

        nav_right_layout.addLayout(nav_duration_row)
        nav_right_layout.addWidget(self.navigation_duration_spin)

        # Cruise toggle
        self.btn_cruise_toggle = QPushButton("CRUISE: OFF")
        self.btn_cruise_toggle.setCheckable(True)
        self.btn_cruise_toggle.setStyleSheet("border: 2px solid #FF4500; color: #FF4500;")
        self.btn_cruise_toggle.clicked.connect(self.toggle_cruise)
        nav_right_layout.addWidget(self.btn_cruise_toggle)

        # --- Status indicators (Navigation Tab) ---
        self.nav_imu_health_label = QLabel("IMU HEALTH: UNKNOWN")
        self.nav_imu_health_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
        self.nav_servo_status_label = QLabel("SERVO DRIVER STATUS: UNKNOWN")
        self.nav_servo_status_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
        self.nav_servo_status_label.setWordWrap(True)

        self.nav_roll_pid_status_label = QLabel("ROLL PID: UNKNOWN")
        self.nav_roll_pid_status_label.setStyleSheet(
            "font-size: 18px; color: #AAAAAA;"
        )
        self.nav_pitch_pid_status_label = QLabel("PITCH PID: UNKNOWN")
        self.nav_pitch_pid_status_label.setStyleSheet(
            "font-size: 18px; color: #AAAAAA;"
        )

        nav_right_layout.addWidget(self.nav_imu_health_label)
        nav_right_layout.addWidget(self.nav_servo_status_label)
        nav_right_layout.addWidget(self.nav_roll_pid_status_label)
        nav_right_layout.addWidget(self.nav_pitch_pid_status_label)
        nav_right_layout.addStretch(1)
        nav_main_layout.addLayout(nav_right_layout, 1)

        navigation_layout.addLayout(nav_main_layout)
        navigation_tab.setLayout(navigation_layout)

        # === OPERATION TAB ===
        top_status_row = QHBoxLayout()
        self.imu_health_label = QLabel("IMU HEALTH: UNKNOWN")
        self.imu_health_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
        self.servo_status_label = QLabel("SERVO DRIVER STATUS: UNKNOWN")
        self.servo_status_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
        self.servo_status_label.setWordWrap(True)
        top_status_row.addWidget(self.imu_health_label)
        top_status_row.addWidget(self.servo_status_label)
        operation_layout.addLayout(top_status_row)

        self.heading_hud = HeadingBarWidget()
        self.heading_hud.setMaximumHeight(80)
        operation_layout.addWidget(self.heading_hud, alignment=Qt.AlignTop)

        self.attitude_widget = AttitudeIndicator()
        operation_layout.addWidget(self.attitude_widget)
        self.ros_node.attitude_widget = self.attitude_widget

        main_layout = QHBoxLayout()

        # LEFT COLUMN
        left_layout = QVBoxLayout()
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
        self.btn_canned.clicked.connect(self.send_canned_and_remember)
        self.btn_duration_increase.clicked.connect(self.increase_duration)
        self.btn_duration_decrease.clicked.connect(self.decrease_duration)

        left_layout.addWidget(self.btn_pid_toggle)
        left_layout.addWidget(self.btn_canned)
        left_layout.addWidget(self.btn_duration_increase)
        left_layout.addWidget(self.btn_duration_decrease)
        left_layout.addWidget(self.btn_toggle_sticky)
        left_layout.addStretch(1)
        main_layout.addLayout(left_layout, 1)

        # RIGHT COLUMN
        right_layout = QVBoxLayout()
        self.virtual_joystick = VirtualJoystickWidget()
        self.virtual_joystick.callback = self.joystick_callback

        right_layout.addWidget(self.virtual_joystick)
        right_layout.addStretch(1)
        main_layout.addLayout(right_layout, 1)

        operation_layout.addLayout(main_layout)

        # === BOTTOM STATUS TABS ===
        status_tabs = QTabWidget()
        status_tabs.setAttribute(Qt.WA_StyledBackground, True)
        status_tabs.setStyleSheet("background: transparent;")

        # CONTROL STATUS TAB
        control_status_tab = QWidget()
        control_status_tab.setAttribute(Qt.WA_StyledBackground, True)
        control_status_tab.setStyleSheet("background: transparent;")
        self.control_status_layout = QVBoxLayout(control_status_tab)
        self.control_status_field = ControlStatusField()
        self.control_status_field.setReadOnly(True)
        self.control_status_field.setWordWrapMode(QTextOption.WordWrap)
        self.control_status_field.setLineWrapMode(QTextEdit.WidgetWidth)
        self.control_status_layout.addWidget(self.control_status_field)
        status_tabs.addTab(control_status_tab, "CONTROL STATUS")

        # SYSTEM STATUS TAB
        system_status_tab = QWidget()
        system_status_tab.setAttribute(Qt.WA_StyledBackground, True)
        system_status_tab.setStyleSheet("background: transparent;")
        system_status_layout = QVBoxLayout(system_status_tab)
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        system_status_layout.addWidget(self.status_display)
        status_tabs.addTab(system_status_tab, "SYSTEM STATUS")
        
        # === TRANSPARENCY FIX FOR QTextEdit FIELDS ===
        self.control_status_field.setStyleSheet("background: transparent;")
        self.control_status_field.setAttribute(Qt.WA_TranslucentBackground, True)
        self.control_status_field.viewport().setAutoFillBackground(False)
        self.control_status_field.viewport().setStyleSheet("background: transparent;")

        self.status_display.setStyleSheet("background: transparent;")
        self.status_display.setAttribute(Qt.WA_TranslucentBackground, True)
        self.status_display.viewport().setAutoFillBackground(False)
        self.status_display.viewport().setStyleSheet("background: transparent;")


        # ===  THE FIX: TRANSPARENCY FOR STATUS FIELDS ===
        self.control_status_field.setStyleSheet("""
            border: 2px solid #00FFFF;
            color: #00FF00;
        """)
        self.control_status_field.setAutoFillBackground(False)
        self.control_status_field.viewport().setAutoFillBackground(False)
        self.control_status_field.viewport().setStyleSheet("background-color: transparent;")

        self.status_display.setStyleSheet("""
            border: 2px solid #00FFFF;
            color: #00FF00;
        """)
        self.status_display.setAutoFillBackground(False)
        self.status_display.viewport().setAutoFillBackground(False)
        self.status_display.viewport().setStyleSheet("background-color: transparent;")

        # === Tab Styling ===
        status_tabs.setStyleSheet("""
            QTabBar::tab {
                height: 40px;
                width: 252px;
                font-size: 18px;
                background-color: rgba(34, 34, 34, 180);
                border: 2px solid #00FFFF;
                margin-right: 4px;
            }
            QTabBar::tab:selected {
                background-color: rgba(51, 51, 51, 200);
                border-bottom: 4px solid #FF4500;
            }
        """)

        operation_layout.addWidget(status_tabs)
        operation_tab.setLayout(operation_layout)

        self.update_lifecycle_buttons()


        
    def adjust_control_status_height(self):
        doc_height = self.control_status_field.document().size().height()
        padding = 10  # You can tweak this padding if needed
        self.control_status_field.setMinimumHeight(int(doc_height + padding))



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

    def toggle_cruise(self):
        enabled = self.btn_cruise_toggle.isChecked()
        self.ros_node.cruise_enabled = enabled
        if enabled:
            self.btn_cruise_toggle.setText("CRUISE: ON")
            self.btn_cruise_toggle.setStyleSheet("border: 2px solid #00FF00; color: #00FF00;")
        else:
            self.btn_cruise_toggle.setText("CRUISE: OFF")
            self.btn_cruise_toggle.setStyleSheet("border: 2px solid #FF4500; color: #FF4500;")

    def toggle_manual_feedback(self):
        self.manual_feedback_enabled = not self.manual_feedback_enabled
        if self.manual_feedback_enabled:
            self.btn_manual_feedback_toggle.setText("STEP FEEDBACK: ON")
            self.btn_manual_feedback_toggle.setStyleSheet(
                "border: 2px solid #00FF00; color: #00FF00;"
            )
        else:
            self.btn_manual_feedback_toggle.setText("STEP FEEDBACK: OFF")
            self.btn_manual_feedback_toggle.setStyleSheet(
                "border: 2px solid #FF4500; color: #FF4500;"
            )
            if self.last_manual_button:
                self.last_manual_button.setStyleSheet("border: 2px solid #00FFFF;")

    def highlight_manual_button(self, btn):
        if self.last_manual_button:
            self.last_manual_button.setStyleSheet("border: 2px solid #00FFFF;")
        btn.setStyleSheet("border: 2px solid #00FF00; color: #00FF00;")
        self.last_manual_button = btn

    def make_manual_step_handler(self, btn, method):
        def handler():
            dur = self.manual_duration_spin.value()
            method(duration_scale=dur)
            self.ros_node.last_canned_callback = lambda: method(duration_scale=dur)
            if self.manual_feedback_enabled:
                self.highlight_manual_button(btn)
        return handler

    def make_nav_step_handler(self, method):
        def handler():
            dur = self.navigation_duration_spin.value()
            method(duration_scale=dur)
            self.ros_node.last_canned_callback = lambda: method(duration_scale=dur)
        return handler

    def send_canned_and_remember(self):
        self.ros_node.publish_canned()
        self.ros_node.last_canned_callback = self.ros_node.publish_canned

        
        
    def quit_app(self):
        QApplication.quit()

    def increase_duration(self):
        self.ros_node.canned_duration_factor += self.ros_node.DURATION_STEP
        #self.ros_node.get_logger().info(f"DURATION FACTOR INCREASED: {self.ros_node.canned_duration_factor:.2f}")

    def decrease_duration(self):
        new_factor = self.ros_node.canned_duration_factor - self.ros_node.DURATION_STEP
        self.ros_node.canned_duration_factor = max(0.1, new_factor)
        #self.ros_node.get_logger().info(f"DURATION FACTOR DECREASED: {self.ros_node.canned_duration_factor:.2f}")


    def update_manual_duration_label(self):
        self.manual_duration_label.setText(f"{self.manual_duration_spin.value():.1f}s")

    def update_nav_duration_label(self):
        self.navigation_duration_label.setText(f"{self.navigation_duration_spin.value():.1f}s")

    def joystick_callback(self, norm_x, norm_y):
        max_angle = 15.0  # or change to 45.0 for tighter control

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

    def nav_joystick_callback(self, norm_x, norm_y):
        # Incremental control: down -> pitch up
        delta_pitch = -norm_y * 5
        delta_roll = norm_x * 5

        if not hasattr(self, 'last_nav_joystick_publish'):
            self.last_nav_joystick_publish = time.time()
        now = time.time()
        if now - self.last_nav_joystick_publish < 0.05:
            return
        self.last_nav_joystick_publish = now

        self.ros_node.target_pitch = max(-100, min(100, self.ros_node.target_pitch + delta_pitch))
        self.ros_node.target_roll = max(-100, min(100, self.ros_node.target_roll + delta_roll))

        self.ros_node.publish_pitch()
        self.ros_node.publish_roll()

    def update_status(self):
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

        servo_text = ""
        if self.ros_node.current_servo_angles:
            servo_text = "|".join(
                [f"<b>S{i+1}:</b> {angle:.1f}°" for i, angle in enumerate(self.ros_node.current_servo_angles)]
            )
        else:
            servo_text = "N/A"

        factor = self.ros_node.canned_duration_factor
        if factor < 1.0:
            factor_str = f'<font color="#FF4500">{factor:.2f}</font>'
        else:
            factor_str = f'{factor:.2f}'

        control_status = (
            f"PITCH COMMAND: {colorize(pitch_cmd)}<br>"
            f"ROLL COMMAND: {colorize(roll_cmd)}<br>"
            f"CURRENT PITCH: {colorize(current_pitch)}<br>"
            f"CURRENT ROLL: {colorize(current_roll)}<br>"
            f"SERVO ANGLES:<br>{servo_text}<br>"
            f"CANNED DURATION FACTOR: {factor_str}<br>"
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
            self.adjust_control_status_height()

            self.last_control_status = control_status

        if getattr(self, 'last_overall_status', None) != overall_status:
            self.status_display.setHtml(overall_status)
            self.last_overall_status = overall_status

        # === Always Update Heading Widget ===
        if self.ros_node.heading is not None:
            self.heading_hud.update_heading_target(self.ros_node.heading)

        # === Always Update Attitude Widget (Pose) ===
        if self.ros_node.euler is not None and len(self.ros_node.euler) >= 3:
            self.attitude_widget.update_attitude_target(self.ros_node.euler)
            if hasattr(self, 'nav_attitude_widget'):
                self.nav_attitude_widget.update_attitude_target(self.ros_node.euler)

            self.attitude_widget.update()
            if hasattr(self, 'nav_attitude_widget'):
                self.nav_attitude_widget.update()
            
                        # === Update Depth on the Attitude Widget ===
        if hasattr(self.ros_node, 'depth'):
            self.attitude_widget.set_depth(self.ros_node.depth)
            if hasattr(self, 'nav_attitude_widget'):
                self.nav_attitude_widget.set_depth(self.ros_node.depth)



        # === Update PID Toggle Button State ===
        self.update_pid_button_state()
        
        # === Update IMU Health Label ===
        imu_health = self.ros_node.imu_health_status
        normalized = imu_health.upper()

        if normalized == "IMU OK":
            self.imu_health_label.setText("IMU HEALTH: OK")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #00FF00;")
            if hasattr(self, 'nav_imu_health_label'):
                self.nav_imu_health_label.setText("IMU HEALTH: OK")
                self.nav_imu_health_label.setStyleSheet("font-size: 18px; color: #00FF00;")
        elif "UNSTABLE" in normalized:
            self.imu_health_label.setText(f"IMU HEALTH: {imu_health}")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #FFA500;")  # Orange
            if hasattr(self, 'nav_imu_health_label'):
                self.nav_imu_health_label.setText(f"IMU HEALTH: {imu_health}")
                self.nav_imu_health_label.setStyleSheet("font-size: 18px; color: #FFA500;")
        elif "RESTARTING" in normalized:
            self.imu_health_label.setText(f"IMU HEALTH: {imu_health}")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #FF4500;")  # Red
            if hasattr(self, 'nav_imu_health_label'):
                self.nav_imu_health_label.setText(f"IMU HEALTH: {imu_health}")
                self.nav_imu_health_label.setStyleSheet("font-size: 18px; color: #FF4500;")
        else:
            # Display unrecognised status text directly
            self.imu_health_label.setText(f"IMU HEALTH: {imu_health or 'UNKNOWN'}")
            self.imu_health_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
            if hasattr(self, 'nav_imu_health_label'):
                self.nav_imu_health_label.setText(f"IMU HEALTH: {imu_health or 'UNKNOWN'}")
                self.nav_imu_health_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
            
        servo_status = self.ros_node.servo_driver_status
        self.servo_status_label.setText(f"SERVO DRIVER STATUS: {servo_status}")
        if hasattr(self, 'nav_servo_status_label'):
            self.nav_servo_status_label.setText(f"SERVO DRIVER STATUS: {servo_status}")

        if "NOMINAL" in servo_status.upper():
            self.servo_status_label.setStyleSheet("font-size: 18px; color: #00FF00;")  # Green
            if hasattr(self, 'nav_servo_status_label'):
                self.nav_servo_status_label.setStyleSheet("font-size: 18px; color: #00FF00;")
        elif "BUSY" in servo_status.upper():
            self.servo_status_label.setStyleSheet("font-size: 18px; color: #FFA500;")  # Yellow / Orange
            if hasattr(self, 'nav_servo_status_label'):
                self.nav_servo_status_label.setStyleSheet("font-size: 18px; color: #FFA500;")
        elif "UNKNOWN" in servo_status.upper():
            self.servo_status_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")  # Gray
            if hasattr(self, 'nav_servo_status_label'):
                self.nav_servo_status_label.setStyleSheet("font-size: 18px; color: #AAAAAA;")
        else:
            self.servo_status_label.setStyleSheet("font-size: 18px; color: #FF4500;")  # Red for anything else (errors, faults)
            if hasattr(self, 'nav_servo_status_label'):
                self.nav_servo_status_label.setStyleSheet("font-size: 18px; color: #FF4500;")

        # === Update PID Status Labels (Navigation Tab) ===
        if hasattr(self, 'nav_roll_pid_status_label'):
            if self.ros_node.roll_pid_enabled:
                self.nav_roll_pid_status_label.setText("ROLL PID: ACTIVE")
                self.nav_roll_pid_status_label.setStyleSheet(
                    "font-size: 18px; color: #00FF00;"
                )
            else:
                self.nav_roll_pid_status_label.setText("ROLL PID: INACTIVE")
                self.nav_roll_pid_status_label.setStyleSheet(
                    "font-size: 18px; color: #FF4500;"
                )

        if hasattr(self, 'nav_pitch_pid_status_label'):
            if self.ros_node.tail_pid_enabled:
                self.nav_pitch_pid_status_label.setText("PITCH PID: ACTIVE")
                self.nav_pitch_pid_status_label.setStyleSheet(
                    "font-size: 18px; color: #00FF00;"
                )
            else:
                self.nav_pitch_pid_status_label.setText("PITCH PID: INACTIVE")
                self.nav_pitch_pid_status_label.setStyleSheet(
                    "font-size: 18px; color: #FF4500;"
                )
            




                        
                

            
    def update_lifecycle_buttons(self):
        current_state = self.ros_node.current_lifecycle_state

        # List of all managed buttons
        all_buttons = [
            self.btn_configure,
            self.btn_activate,
            self.btn_deactivate,
            self.btn_cleanup,
            self.btn_shutdown,
            self.btn_error_recovery,
        ]

        # If no node is available, disable everything
        if not current_state or current_state == "unavailable":
            for btn in all_buttons:
                btn.setEnabled(False)
                btn.setStyleSheet("border: 2px solid #555555; color: #555555;")
            self.label_current_state.setText("CURRENT STATE: NODE OFFLINE")
            return

        # Mapping of lifecycle states to buttons that should be enabled
        state_map = {
            "unconfigured": [self.btn_configure, self.btn_shutdown],
            "inactive": [self.btn_activate, self.btn_cleanup, self.btn_shutdown],
            "active": [self.btn_deactivate, self.btn_shutdown],
            "errorprocessing": [self.btn_error_recovery],
        }

        # Disable all by default
        for btn in all_buttons:
            btn.setEnabled(False)
            btn.setStyleSheet("border: 2px solid #555555; color: #555555;")

        # Enable only the allowed buttons for the current state
        for btn in state_map.get(current_state.lower(), []):
            btn.setEnabled(True)
            btn.setStyleSheet("border: 2px solid #00FF00; color: #00FF00;")

        self.label_current_state.setText(f"CURRENT STATE: {current_state.upper()}")

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


        

