
"""Predefined movement sequences for the AUV."""

from auv_custom_interfaces.msg import ServoMovementCommand
import rclpy
import math


class CannedMovements:
    """Publish canned servo movements through a ROS interface."""

    def __init__(self, ros_interface):
        self.ros = ros_interface

    def _prepend_current_positions(self, commands, base_duration: float = 1.0):
        """Prepend current servo positions to command sequence.

        This ensures each canned movement starts from the most recently
        reported servo angles. If current angles are unavailable the
        commands are returned unchanged.
        """

        servo_nums = commands.get('servo_numbers', [])
        if not servo_nums:
            return commands

        current = getattr(self.ros, 'current_servo_angles', None)
        if not current or len(current) <= max(servo_nums):
            # Skip prepend if we don't have angles for all requested servos
            return commands

        try:
            current_angles = [current[n] for n in servo_nums]
        except Exception:
            return commands

        commands['target_angles'] = current_angles + commands['target_angles']
        commands['durations'] = [base_duration] + commands['durations']
        commands['easing_algorithms'] = ['EXPONENTIAL'] + commands['easing_algorithms']
        commands['easing_in_factors'] = [0.0] + commands['easing_in_factors']
        commands['easing_out_factors'] = [0.0] + commands['easing_out_factors']

        return commands

    def _fill_current_placeholders(self, commands):
        """Replace ``None`` or NaN target angles with current servo angles."""
        servo_nums = commands.get('servo_numbers', [])
        if not servo_nums:
            return commands

        angles = list(commands.get('target_angles', []))
        current = getattr(self.ros, 'current_servo_angles', None)
        if not current or len(current) <= max(servo_nums):
            return commands

        step = len(servo_nums)
        for i, angle in enumerate(angles):
            if angle is None or (isinstance(angle, float) and math.isnan(angle)):
                servo_idx = servo_nums[i % step]
                try:
                    angles[i] = float(current[servo_idx])
                except Exception:
                    pass
        commands['target_angles'] = angles
        return commands

    def _publish(self, commands):
        commands = self._prepend_current_positions(commands)
        commands = self._fill_current_placeholders(commands)
        servo_nums = commands['servo_numbers']
        wing_used = any(n in [0, 1, 2, 3] for n in servo_nums)
        tail_used = any(n in [4, 5] for n in servo_nums)

        if wing_used:
            self.ros.set_pid(False)
            self.ros.roll_pid_enabled = False
            self.ros.pid_reattach_pending = True

        if tail_used:
            self.ros.set_tail_pid(False)
            self.ros.tail_pid_enabled = False
            self.ros.tail_pid_reattach_pending = True

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
        self.ros.last_movement_type = commands['movement_type']
        self.ros.last_command = (
            f"CANNED MOVEMENT PUBLISHED @ {self.ros.get_clock().now().to_msg()}"
        )

    def canned_1_low_thrust(self, duration_scale: float = 1.0): # minimum thrust
        base_durations = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 180.0, 90.0, 90.0, #pitch up
                             0.0, 180.0, 180.0, 90.0, # Swing up
                             0.0, 135.0, 180.0, 135.0, # Stay Up and glide
                             0.0, 90.0, 180.0, 180.0, # Pitch Down
                             180.0, 90.0, 0.0, 180.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide
                             180.0, 180.0, 0.0, 90.0, # Pitch up
                             90.0, 180.0, 90.0, 90.0, # Swing back to glide
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'movement_type': 'THRUST_UNIT_1',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_2_medium_thrust(self, duration_scale: float = 1.0): # Medium Thrust
        base_durations = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 165.0, 90.0, 105.0, #pitch up
                             0.0, 165.0, 180.0, 105.0, # Swing up
                             0.0, 135.0, 180.0, 135.0, # Stay Up and glide
                             0.0, 105.0, 180.0, 165.0, # Pitch Down
                             180.0, 105.0, 0.0, 165.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide
                             180.0, 165.0, 0.0, 105.0, # Pitch up
                             90.0, 165.0, 90.0, 105.0, # Swing back to glide
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT_2',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_3_high_thrust(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 150.0, 90.0, 120.0, #pitch up
                             0.0, 150.0, 180.0, 120.0, # Swing up
                             0.0, 135.0, 180.0, 135.0, # Stay Up and glide
                             0.0, 120.0, 180.0, 150.0, # Pitch Down
                             180.0, 120.0, 0.0, 150.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide
                             180.0, 150.0, 0.0, 120.0, # Pitch up
                             90.0, 150.0, 90.0, 120.0, # Swing back to glide
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT_3',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_4_forward_right(self, duration_scale: float = 1.0):# both wings swing, but with different attack angles. Using medium thrust on left wing, and low thrust in right wing should yaw right
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 165.0, 90.0, 90.0, #pitch up
                             0.0, 165.0, 180.0, 90.0, # Swing up
                             0.0, 135.0, 180.0, 135.0, # Stay Up and glide
                             0.0, 105.0, 180.0, 180.0, # Pitch Down
                             180.0, 105.0, 0.0, 180.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide
                             180.0, 165.0, 0.0, 90.0, # Pitch up
                             90.0, 165.0, 90.0, 90.0, # Swing back to glide
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'FORWARD_RIGHT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_5_forward_left(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 180.0, 90.0, 105.0, #pitch up
                             0.0, 180.0, 180.0, 105.0, # Swing up
                             0.0, 135.0, 180.0, 135.0, # Stay Up and glide
                             0.0, 90.0, 180.0, 165.0, # Pitch Down
                             180.0, 90.0, 0.0, 165.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide
                             180.0, 180.0, 0.0, 105.0, # Pitch up
                             90.0, 180.0, 90.0, 105.0, # Swing back to glide
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'FORWARD_LEFT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_6_hard_right(self, duration_scale: float = 1.0): #only left side produces thrust, should yaw right. Using low thrust on other wing
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 180.0, 90.0, 135.0, #pitch up
                             0.0, 180.0, 90.0, 135.0, # Swing up
                             0.0, 135.0, 90.0, 135.0, # Stay Up and glide
                             0.0, 90.0, 90.0, 135.0, # Pitch Down
                             180.0, 90.0, 90.0, 135.0, # Swing down
                             180.0, 135.0, 90.0, 135.0, # Stay down and glide
                             180.0, 180.0, 90.0, 135.0, # Pitch up
                             90.0, 180.0, 90.0, 135.0, # Swing back to glide
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'HARD_RIGHT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_7_hard_left(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 135.0, 90.0, 90.0, #pitch up
                             90.0, 135.0, 180.0, 90.0, # Swing up
                             90.0, 135.0, 180.0, 135.0, # Stay Up and glide
                             90.0, 135.0, 180.0, 180.0, # Pitch Down
                             90.0, 135.0, 0.0, 180.0, # Swing down
                             90.0, 135.0, 0.0, 135.0, # Stay down and glide
                             90.0, 135.0, 0.0, 90.0, # Pitch up
                             90.0, 135.0, 90.0, 90.0, # Swing back to glide
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'HARD_LEFT',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_8_tail_thrust(self, duration_scale: float = 1.0):
        base_durations = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]# will use 0.5 for now, should have same amount of items as target angles below
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [4, 5],
            'target_angles': [50.0, 50.0,
                             130.0, 130.0,
                             50.0, 50.0,
                             130.0, 130.0,
                             50.0, 50.0,
                             130.0, 130.0,
                             50.0, 50.0,
                             130.0, 130.0,
                             50.0, 50.0,],
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'TAIL_THRUST',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_9_SWING_UP(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [None, 165.0, None, 105.0, #pitch up
                             0.0, 165.0, 180.0, 105.0, # Swing up
                             0.0, 135.0, 180.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0],
            'movement_type': 'SWING_UP',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_10_DOWN_TO_GLIDE(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [None, 105.0, None, 165.0, # Pitch Down
                             90.0, 105.0, 90.0, 165.0, # Swing down
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0],
            'movement_type': 'DOWN_TO_GLIDE',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_11_UP_TO_GLIDE(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [None, 165.0, None, 105.0, # Pitch Up
                             90.0, 165.0, 90.0, 105.0, # Swing up
                             90.0, 135.0, 90.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0],
            'movement_type': 'UP_TO_GLIDE',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    def canned_12_SWING_DOWN(self, duration_scale: float = 1.0):
        base_durations = [0.2, 2.0, 0.1]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [None, 105.0, None, 165.0, # Pitch DOWN
                             180.0, 105.0, 0.0, 165.0, # Swing DOWN
                             180.0, 135.0, 0.0, 135.0], # Pitch to neutral
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'
            ],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0],
            'movement_type': 'SWING_DOWN',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)

    
    def canned_13_ACCEL(self, duration_scale: float = 1.0): # minimum thrust
        base_durations = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,]
        adjusted = [
            d * self.ros.canned_duration_factor * duration_scale
            for d in base_durations
        ]
        commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [90.0, 180.0, 90.0, 90.0, #pitch up from glide
                             0.0, 180.0, 180.0, 90.0, # Swing up
                             0.0, 135.0, 180.0, 135.0, # Stay Up and glide
                             0.0, 90.0, 180.0, 180.0, # Pitch Down
                             180.0, 90.0, 0.0, 180.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide 
                             180.0, 180.0, 0.0, 90.0, # Pitch up
                             0.0, 180.0, 180.0, 90.0, # Swing to top, same pitch
                             0.0, 135.0, 180.0, 135.0, #pitch to neutral - end accel one
                             0.0, 105.0, 180.0, 165.0, # Pitch Down ## start thrust 2
                             180.0, 105.0, 0.0, 165.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide
                             180.0, 165.0, 0.0, 105.0, # Pitch up
                             0.0, 165.0, 180.0, 105.0, # swing up
                             0.0, 135.0, 180.0, 135.0, # up and glide ### end thrust 2
                             0.0, 120.0, 180.0, 150.0, # Pitch Down
                             180.0, 120.0, 0.0, 150.0, # Swing down
                             180.0, 135.0, 0.0, 135.0, # Stay down and glide
                             180.0, 150.0, 0.0, 120.0, # Pitch up
                             0.0, 150.0, 180.0, 120.0, # Swing back to top
                             0.0, 135.0, 180.0, 135.0, # up and glide ### end thrust 3
                             0.0, 120.0, 180.0, 150.0, # Pitch Down
                             90.0, 120.0, 90.0, 150.0, #swing to glide
                             90.0, 135.0, 90.0, 135.0], #glide ##24 movements
            'durations': adjusted,
            'easing_algorithms': [
                'EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL',
                'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 
                'EXPONENTIAL', 'EXPONENTIAL', 'CUBIC', 'CUBIC',
                'EXPONENTIAL', 'CUBIC', 'CUBIC','EXPONENTIAL', 
                'CUBIC', 'EXPONENTIAL', 'EXPONENTIAL', 'CUBIC', 
                'CUBIC', 'EXPONENTIAL','CUBIC', 'CUBIC', 
            ],
            'easing_in_factors': [0.0] * 24,
            'easing_out_factors': [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,],
            'movement_type': 'ACCELERATION',
            'deadline': (self.ros.get_clock().now() +
                        rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0,
        }
        self._publish(commands)
