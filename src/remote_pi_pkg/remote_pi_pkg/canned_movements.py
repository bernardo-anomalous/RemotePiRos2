class CannedMovements:
    """Helper exposing one method per canned movement step."""

    def __init__(self, ros_interface):
        self._ros = ros_interface

    def _send(self, index, duration=None):
        if duration is None:
            duration = self._ros.canned_default_durations[index]
        self._ros.publish_canned_step(index, duration)

    def canned_pitch_up(self, duration=None):
        self._send(0, duration)

    def canned_swing_up(self, duration=None):
        self._send(1, duration)

    def canned_up_glide(self, duration=None):
        self._send(2, duration)

    def canned_pitch_down(self, duration=None):
        self._send(3, duration)

    def canned_swing_down(self, duration=None):
        self._send(4, duration)

    def canned_down_glide(self, duration=None):
        self._send(5, duration)

    def canned_pitch_up_2(self, duration=None):
        self._send(6, duration)

    def canned_wing_to_glide(self, duration=None):
        self._send(7, duration)

    def canned_glide(self, duration=None):
        self._send(8, duration)
