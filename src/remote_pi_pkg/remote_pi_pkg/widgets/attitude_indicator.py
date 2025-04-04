from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QTransform
from PyQt5.QtCore import Qt, QPointF

class AttitudeIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumHeight(250)
        self.setMinimumWidth(200)

        # === Customizable Style Parameters ===
        self.base_width = 60
        # Triangle base half-width
        self.pitch_effect_scale = 1.0
        self.max_angle = 90.0  # Maximum pitch/roll for normalization

        self.outer_color = QColor("#00FFFF")    # Aqua (pose triangle)
        self.intensity_color = QColor("#FF4500")  # Orange-red (intensity fill)
        self.horizon_color = QColor("#00FF00")  # Green (horizon lines)

    def update_attitude(self, euler):
        if euler and len(euler) >= 2:
            try:
                self.roll = float(euler[0])
                self.pitch = float(euler[1])
                self.update()
            except Exception:
                pass

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), Qt.transparent)

        center = self.rect().center()
        width = self.rect().width()

        # Normalize pitch and roll to range -1 to 1
        norm_pitch = max(min(self.pitch / self.max_angle, 1.0), -1.0)
        norm_roll = max(min(self.roll / self.max_angle, 1.0), -1.0)
        max_intensity = max(abs(norm_pitch), abs(norm_roll))

        # === Triangle Dimensions ===
        base_half_width = self.base_width  # Half the triangle base (left/right)
        equilateral_height = base_half_width * (3**0.5)  # Full height at 90° pitch

        # Morph from flat (0 height) to equilateral (max height) based on pitch
        nose_y = -equilateral_height * norm_pitch  # Invert Y to match natural pitch sense

        # Triangle points (centered at origin)
        point_nose = QPointF(0, nose_y)
        point_left = QPointF(-base_half_width, 0)
        point_right = QPointF(base_half_width, 0)
        triangle = [point_nose, point_left, point_right]

        # Transform to center of widget and apply roll
        transform = QTransform()
        transform.translate(center.x(), center.y())
        transform.rotate(self.roll)
        transformed = [transform.map(p) for p in triangle]

        # Draw pose triangle
        painter.setBrush(QBrush(self.outer_color))
        painter.setPen(QPen(self.outer_color, 2))
        painter.drawPolygon(*transformed)

        # Draw intensity triangle (scaled)
        inner_scale = 0.1 + 0.9 * max_intensity
        inner_triangle = [QPointF(p.x() * inner_scale, p.y() * inner_scale) for p in triangle]
        inner_transformed = [transform.map(p) for p in inner_triangle]

        painter.setBrush(QBrush(self.intensity_color))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(*inner_transformed)

        # Horizon markers
        painter.setPen(QPen(self.horizon_color, 2))
        painter.drawLine(10, center.y(), 30, center.y())
        painter.drawLine(width - 30, center.y(), width - 10, center.y())

