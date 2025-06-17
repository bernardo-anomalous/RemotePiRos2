
"""Predefined movement sequences for the AUV."""

from auv_custom_interfaces.msg import ServoMovementCommand
import rclpy


class CannedMovements:
    """Publish canned servo movements through a ROS interface."""

    def __init__(self, ros_interface):
        self.ros = ros_interface

    def _publish(self, commands):
        self.ros.set_pid(False)
        self.ros.roll_pid_enabled = False
        self.ros.pid_reattach_pending = True

        msg = ServoMovementCommand()
        msg.header.stamp = self.ros.get_clock().now().to_msg()
        msg.servo_numbers = commands['servo_numbers']
        msg.target_angles = commands['target_angles']
        msg.durations = commands['durations']
        msg.easing_algorithms = commands['easing_algorithms']
        msg.easing_in_factors = commands['easing_in_factors']
        msg.easing_out_factors = commands['easing_out_factors']
        msg.movement_type = commands['movement_type']
        msg.deadline = commands['deadline']
        msg.operational_mode = commands['operational_mode']
        msg.priority = commands['priority']

        self.ros.canned_pub.publish(msg)
        self.ros.last_command = (
            f"CANNED MOVEMENT PUBLISHED @ {self.ros.get_clock().now().to_msg()}"
        )

    def canned_1(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_2(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_3(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_4(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_5(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_6(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_7(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_8(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_9(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_10(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 140.0, 90.0, 40.0,
                             0.0, 140.0, 180.0, 40.0,
                             0.0, 90.0, 180.0, 90.0,
                             0.0, 40.0, 180.0, 140.0,
                             180.0, 40.0, 0.0, 140.0,
                             180.0, 90.0, 0.0, 90.0,
                             180.0, 140.0, 0.0, 40.0,
                             90.0, 140.0, 90.0, 40.0,
                             90.0, 90.0, 90.0, 90.0],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

