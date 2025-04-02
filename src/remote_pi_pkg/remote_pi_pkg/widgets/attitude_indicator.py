from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtCore import QPoint

class AttitudeIndicator(QWidget):
    """
    A simple attitude indicator that draws an artificial horizon.
    Uses Euler angles [roll, pitch, yaw] to shift a horizon line.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update_attitude(self, euler):
        if euler and len(euler) >= 3:
            try:
                self.roll = float(euler[0])
                self.pitch = float(euler[1])
                self.yaw = float(euler[2])
            except Exception:
                self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        painter.fillRect(rect, QColor("#141414"))
        center = rect.center()

        # Calculate pitch offset
        pitch_offset = self.pitch * 2  # scale factor
        line_length = rect.width() * 1.5
        start = QPoint(center.x() - line_length // 2, center.y() + pitch_offset)
        end = QPoint(center.x() + line_length // 2, center.y() + pitch_offset)

        # Rotate the horizon according to roll
        painter.translate(center)
        painter.rotate(-self.roll)
        painter.translate(-center)

        pen = QPen(QColor("#00FF00"), 3)
        painter.setPen(pen)
        painter.drawLine(start, end)
        painter.drawEllipse(center, 5, 5)
        painter.resetTransform()
