import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, String
from auv_custom_interfaces.msg import ServoMovementCommand
from remote_pi_pkg import CannedMovements
import threading


class GamepadMapper(Node):
    """Map joystick input to AUV control topics."""

    def __init__(self):
        super().__init__('gamepad_mapper')
        self.target_roll_pub = self.create_publisher(Float32, 'target_roll', 10)
        self.target_pitch_pub = self.create_publisher(Float32, 'target_pitch', 10)
        # Publishers for manual and canned servo commands
        self.servo_pub = self.create_publisher(
            ServoMovementCommand, 'servo_interpolation_commands', 10)
        self.canned_pub = self.servo_pub
        self.wing_pid_pub = self.create_publisher(Bool, 'wing_pid_active', 10)
        self.tail_pid_pub = self.create_publisher(Bool, 'tail_pid_active', 10)
        self.cruise_enabled_pub = self.create_publisher(Bool, 'cruise_enabled', 10)
        self.cruise_delay_pub = self.create_publisher(Float32, 'cruise_delay', 10)
        self.duration_factor_pub = self.create_publisher(
            Float32, 'canned_duration_factor', 10)
        self.step_duration_pub = self.create_publisher(
            Float32, 'step_duration', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.last_buttons = []
        self.cruise_enabled = False
        self.cruise_delay = 2.0
        self.canned_duration_factor = 1.0
        self.step_duration = 1.0
        self.roll_pid_enabled = True
        self.pid_reattach_pending = False
        self.tail_pid_enabled = True
        self.tail_pid_reattach_pending = False
        self.auto_pid_reengage = True
        self.last_servo_status_word = None
        self.servo_driver_status = "UNKNOWN"
        self.last_canned_callback = None
        self.cruise_timer = None
        self.last_command = ""

        self.canned_movements = CannedMovements(self)
        self.create_subscription(String, 'servo_driver_status',
                                 self.servo_status_callback, 10)

        # Map joystick button indices to handler methods. Indices refer to the
        # order provided by the Joy message from the controller.
        self.button_actions = {
            0: self._make_canned_handler(
                self.canned_movements.canned_13_ACCEL),       # A button
            1: self._make_canned_handler(
                self.canned_movements.canned_3_high_thrust),    # B button
            2: self._make_canned_handler(
                self.canned_movements.canned_2_medium_thrust),      # X button
            3: self._make_canned_handler(
                self.canned_movements.canned_1_low_thrust),    # Y button
            
            4: self.decrease_cruise_delay,                # Left bumper
            
            5: self.increase_cruise_delay,                     # Right bumper
            
            6: self.decrease_step_duration,                    # Left Trigger
            
            7: self.increase_step_duration,                     # Right Trigger
            
            8: self._make_canned_handler(
                self.canned_movements.canned_9_SWING_UP),         # Stick press L
            9: self._make_canned_handler(
                self.canned_movements.canned_10_DOWN_TO_GLIDE),   # Stick press R
            10: self._make_canned_handler(
                self.canned_movements.canned_11_UP_TO_GLIDE),     # D-pad up
            11: self._make_canned_handler(
                self.canned_movements.canned_12_SWING_DOWN),      # D-pad down
            12: self._make_canned_handler(
                self.canned_movements.canned_13_ACCEL),           # D-pad left
            13: self.toggle_cruise,                             # D-pad right
            14: self.increase_step_duration,                    # Misc button 1
            15: self.decrease_step_duration,                    # Misc button 2
            16: self.increase_cruise_delay,                     # Misc button 3
        }

    def _make_canned_handler(self, method):
        def handler():
            dur = self.step_duration
            method(duration_scale=dur)
            self.last_canned_callback = lambda: method(duration_scale=dur)
        return handler

    def joy_callback(self, msg: Joy):
        # axes[3] -> roll, axes[4] -> pitch
        if len(msg.axes) > 3:
            roll = msg.axes[3] * 5.0
            pitch = msg.axes[4] * 5.0
            self.target_roll_pub.publish(Float32(data=roll))
            self.target_pitch_pub.publish(Float32(data=pitch))

        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons)

        def pressed(index: int) -> bool:
            return (
                len(msg.buttons) > index
                and msg.buttons[index]
                and not self.last_buttons[index]
            )

        for idx, action in self.button_actions.items():
            if pressed(idx):
                action()

        self.last_buttons = list(msg.buttons)

    def send_servo_command(self, numbers, angles, duration: float = 1.0):
        msg = ServoMovementCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.servo_numbers = numbers
        msg.target_angles = angles
        msg.durations = [duration] * len(angles)
        msg.easing_algorithms = ['LINEAR'] * len(angles)
        msg.easing_in_factors = [0.0] * len(angles)
        msg.easing_out_factors = [0.0] * len(angles)
        msg.movement_type = 'MANUAL'
        msg.deadline = (self.get_clock().now() + rclpy.duration.Duration(seconds=5)).to_msg()
        msg.operational_mode = 'MANUAL'
        msg.priority = 0
        self.servo_pub.publish(msg)

    def set_pid(self, activate: bool):
        self.wing_pid_pub.publish(Bool(data=activate))

    def set_tail_pid(self, activate: bool):
        self.tail_pid_pub.publish(Bool(data=activate))

    def toggle_cruise(self):
        """Toggle cruise mode on/off and publish state."""
        self.cruise_enabled = not self.cruise_enabled
        self.cruise_enabled_pub.publish(Bool(data=self.cruise_enabled))

    def increase_duration_factor(self):
        """Increase canned movement duration scaling."""
        self.canned_duration_factor += 0.2
        self.duration_factor_pub.publish(
            Float32(data=self.canned_duration_factor)
        )

    def increase_step_duration(self):
        """Increase the duration multiplier for canned steps."""
        self.step_duration += 0.1
        self.step_duration_pub.publish(Float32(data=self.step_duration))

    def decrease_duration_factor(self):
        """Decrease canned movement duration scaling."""
        self.canned_duration_factor = max(0.2, self.canned_duration_factor - 0.2)
        self.duration_factor_pub.publish(
            Float32(data=self.canned_duration_factor)
        )

    def decrease_step_duration(self):
        """Decrease the duration multiplier for canned steps."""
        self.step_duration = max(0.1, self.step_duration - 0.1)
        self.step_duration_pub.publish(Float32(data=self.step_duration))

    def decrease_cruise_delay(self):
        """Reduce delay between cruise cycles."""
        self.cruise_delay = max(0.5, self.cruise_delay - 0.5)
        self.cruise_delay_pub.publish(Float32(data=self.cruise_delay))

    def increase_cruise_delay(self):
        """Increase delay between cruise cycles."""
        self.cruise_delay += 0.5
        self.cruise_delay_pub.publish(Float32(data=self.cruise_delay))

    def servo_status_callback(self, msg: String):
        self.servo_driver_status = msg.data
        status_word = msg.data.split(":")[0].strip().lower()
        prev_status = self.last_servo_status_word
        self.last_servo_status_word = status_word

        if status_word == "busy":
            if self.roll_pid_enabled:
                self.set_pid(False)
                self.roll_pid_enabled = False

        elif status_word == "nominal":
            if self.pid_reattach_pending:
                if self.auto_pid_reengage:
                    self.set_pid(True)
                    self.roll_pid_enabled = True
                self.pid_reattach_pending = False

            if self.tail_pid_reattach_pending:
                if self.auto_pid_reengage:
                    self.set_tail_pid(True)
                    self.tail_pid_enabled = True
                self.tail_pid_reattach_pending = False

            if (self.cruise_enabled and prev_status == "busy"
                    and self.last_canned_callback):
                if self.cruise_timer:
                    self.cruise_timer.cancel()
                self.cruise_timer = threading.Timer(
                    self.cruise_delay, self.last_canned_callback)
                self.cruise_timer.start()


def main(args=None):
    rclpy.init(args=args)
    node = GamepadMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
