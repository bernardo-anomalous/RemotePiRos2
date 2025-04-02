from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import QSize, QPoint
from PyQt5.QtGui import QPainter, QPen, QColor
import math

class VirtualJoystickWidget(QWidget):
    """
    A virtual joystick that displays a large circle with crosshairs and a draggable knob.
    The knob's position is normalized to (-1 to 1, -1 to 1) and reported via a callback.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self.knob_radius = 20
        self.dragging = False
        self.knob_pos = self.rect().center()
        self.callback = None  # Callback: function(norm_x, norm_y)
        
        self.sticky_mode = False  # Default to momentary mode


    def sizeHint(self):
        return QSize(300, 300)

    def resizeEvent(self, event):
        self.knob_pos = self.rect().center()
        super().resizeEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        center = rect.center()
        radius = min(rect.width(), rect.height()) // 2 - 10

        pen = QPen(QColor("#00FF00"), 3)
        painter.setPen(pen)
        painter.drawEllipse(center, radius, radius)

        painter.drawLine(center.x() - radius, center.y(), center.x() + radius, center.y())
        painter.drawLine(center.x(), center.y() - radius, center.x(), center.y() + radius)

        painter.setBrush(QColor("#00FF00"))
        painter.drawEllipse(self.knob_pos, self.knob_radius, self.knob_radius)

    def mousePressEvent(self, event):
        if (event.pos() - self.knob_pos).manhattanLength() <= self.knob_radius:
            self.dragging = True

    def mouseMoveEvent(self, event):
        if self.dragging:
            center = self.rect().center()
            max_radius = min(self.rect().width(), self.rect().height()) // 2 - 10 - self.knob_radius
            offset = event.pos() - center
            distance = math.hypot(offset.x(), offset.y())
            if distance > max_radius and distance != 0:
                factor = max_radius / distance
                offset = offset * factor
            self.knob_pos = center + offset
            self.update()
            norm_x = offset.x() / max_radius
            norm_y = -offset.y() / max_radius  # Invert Y: up is positive
            if self.callback:
                self.callback(norm_x, norm_y)

    def mouseReleaseEvent(self, event):
        
        self.dragging = False

        if not self.sticky_mode:
            self.knob_pos = self.rect().center()  # reset to center only in momentary mode

            if self.callback:
                self.callback(0.0, 0.0)
        self.update()

