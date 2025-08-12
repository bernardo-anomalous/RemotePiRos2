import math
import rclpy
from std_msgs.msg import Float32MultiArray
from remote_pi_pkg.ros.gamepad_mapper import GamepadMapper


class DummyPublisher:
    def __init__(self):
        self.last_msg = None

    def publish(self, msg):
        self.last_msg = msg


def test_dpad_canned_movement_uses_current_angles():
    rclpy.init()
    mapper = GamepadMapper()
    mapper.canned_pub = DummyPublisher()

    mapper.current_servo_angles_callback(Float32MultiArray(data=[10.0, 20.0, 30.0, 40.0]))
    mapper.canned_movements.canned_10_DOWN_TO_GLIDE()

    assert mapper.canned_pub.last_msg is not None
    assert all(not math.isnan(angle) for angle in mapper.canned_pub.last_msg.target_angles)

    mapper.destroy_node()
    rclpy.shutdown()
