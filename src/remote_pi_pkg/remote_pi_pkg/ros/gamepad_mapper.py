import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand


class GamepadMapper(Node):
    """Map joystick input to AUV control topics."""

    def __init__(self):
        super().__init__('gamepad_mapper')
        self.target_roll_pub = self.create_publisher(Float32, 'target_roll', 10)
        self.target_pitch_pub = self.create_publisher(Float32, 'target_pitch', 10)
        self.servo_pub = self.create_publisher(
            ServoMovementCommand, 'servo_interpolation_commands', 10)
        self.cruise_enabled_pub = self.create_publisher(Bool, 'cruise_enabled', 10)
        self.cruise_delay_pub = self.create_publisher(Float32, 'cruise_delay', 10)
        self.duration_factor_pub = self.create_publisher(
            Float32, 'canned_duration_factor', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.last_buttons = []
        self.cruise_enabled = False
        self.cruise_delay = 2.0
        self.duration_factor = 1.0

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

        if pressed(0):  # A button -> simple canned 1
            self.send_servo_command([0, 1, 2, 3], [90.0, 180.0, 90.0, 90.0])
        if pressed(1):  # B button -> simple canned 2
            self.send_servo_command([0, 1, 2, 3], [0.0, 165.0, 180.0, 105.0])
        if pressed(2):  # X button -> toggle cruise
            self.cruise_enabled = not self.cruise_enabled
            self.cruise_enabled_pub.publish(Bool(data=self.cruise_enabled))
        if pressed(3):  # Y button -> increase duration factor
            self.duration_factor += 0.2
            self.duration_factor_pub.publish(Float32(data=self.duration_factor))
        if pressed(4):  # LB -> decrease cruise delay
            self.cruise_delay = max(0.5, self.cruise_delay - 0.5)
            self.cruise_delay_pub.publish(Float32(data=self.cruise_delay))
        if pressed(5):  # RB -> increase cruise delay
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


def main(args=None):
    rclpy.init(args=args)
    node = GamepadMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
