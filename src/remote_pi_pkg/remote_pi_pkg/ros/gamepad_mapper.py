import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand
from remote_pi_pkg import CannedMovements


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
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.last_buttons = []
        self.cruise_enabled = False
        self.cruise_delay = 2.0
        self.canned_duration_factor = 1.0
        self.roll_pid_enabled = True
        self.pid_reattach_pending = False
        self.tail_pid_enabled = True
        self.tail_pid_reattach_pending = False
        self.last_command = ""

        self.canned_movements = CannedMovements(self)

    def joy_callback(self, msg: Joy):
        # axes[2] -> roll, axes[3] -> pitch
        if len(msg.axes) > 3:
            roll = msg.axes[2] * 15.0
            pitch = msg.axes[3] * 15.0
            self.target_roll_pub.publish(Float32(data=roll))
            self.target_pitch_pub.publish(Float32(data=pitch))

        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons)

        def pressed(index: int) -> bool:
            return len(msg.buttons) > index and msg.buttons[index] and not self.last_buttons[index]

        canned_methods = [
            self.canned_movements.canned_1_low_thrust,
            self.canned_movements.canned_2_medium_thrust,
            self.canned_movements.canned_3_high_thrust,
            self.canned_movements.canned_4_forward_right,
            self.canned_movements.canned_5_forward_left,
            self.canned_movements.canned_6_hard_right,
            self.canned_movements.canned_7_hard_left,
            self.canned_movements.canned_8_tail_thrust,
            self.canned_movements.canned_9_SWING_UP,
            self.canned_movements.canned_10_DOWN_TO_GLIDE,
            self.canned_movements.canned_11_UP_TO_GLIDE,
            self.canned_movements.canned_12_SWING_DOWN,
            self.canned_movements.canned_13_ACCEL,
        ]

        for idx, method in enumerate(canned_methods):
            if pressed(idx):
                method()

        control_idx = len(canned_methods)
        if pressed(control_idx):
            self.cruise_enabled = not self.cruise_enabled
            self.cruise_enabled_pub.publish(Bool(data=self.cruise_enabled))
        if pressed(control_idx + 1):
            self.canned_duration_factor += 0.2
            self.duration_factor_pub.publish(Float32(data=self.canned_duration_factor))
        if pressed(control_idx + 2):
            self.cruise_delay = max(0.5, self.cruise_delay - 0.5)
            self.cruise_delay_pub.publish(Float32(data=self.cruise_delay))
        if pressed(control_idx + 3):
            self.cruise_delay += 0.5
            self.cruise_delay_pub.publish(Float32(data=self.cruise_delay))

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


def main(args=None):
    rclpy.init(args=args)
    node = GamepadMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
