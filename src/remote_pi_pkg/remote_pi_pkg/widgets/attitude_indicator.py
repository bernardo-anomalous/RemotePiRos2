
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QTransform, QFont
from PyQt5.QtCore import Qt, QPointF, QRectF
import math

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
        
        # === Triangle-to-Pitch-Scale Visual Link ===
        nose_tip = transform.map(point_nose)  # Actual on-screen triangle tip position

        # Define endpoint (stopping short of pitch scale)
        line_end_x = width - 60  # Slightly before the pitch bar
        line_end = QPointF(line_end_x, nose_tip.y())

        # Draw horizontal line from triangle tip to pitch scale
        painter.setPen(QPen(QColor("#00FF00"), 1, Qt.DashLine))  # Dashed green line
        painter.drawLine(nose_tip, line_end)

        # Draw pitch label at end
        painter.setPen(QPen(QColor("#FF4500")))
        painter.setFont(QFont("Courier New", 12, QFont.Bold))
        painter.drawText(line_end + QPointF(5, 5), f"{self.pitch:+.1f}°")


        # Adjust intensity triangle color based on pitch safety range
        if abs(self.pitch) <= 45.0:
            fill_color = QColor("#FFFFFF")  # White = safe
        else:
            fill_color = QColor("#FF4500")  # Red-orange = danger

        inner_scale = 0.1 + 0.9 * max_intensity
        inner_triangle = [QPointF(p.x() * inner_scale, p.y() * inner_scale) for p in triangle]
        inner_transformed = [transform.map(p) for p in inner_triangle]

        # First fill
        painter.setBrush(QBrush(fill_color))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(*inner_transformed)

        # Outline pass
        outline_color = QColor("#00FF00s")  # 💡 Try #335544 or translucent white too
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(outline_color, 3))
        painter.drawPolygon(*inner_transformed)
        


        # Horizon markers
        painter.setPen(QPen(self.horizon_color, 2))
        painter.drawLine(10, center.y(), 30, center.y())
        painter.drawLine(width - 30, center.y(), width - 10, center.y())
        
        # === Roll Arc on Left Side ===
        arc_radius = 100
        arc_center = QPointF(140, center.y())
        arc_rect = QRectF(arc_center.x() - arc_radius,
                        arc_center.y() - arc_radius,
                        arc_radius * 2,
                        arc_radius * 2)

        # Draw arc: start at 90°, sweep 180° CCW (left half)
        painter.setPen(QPen(QColor("#00FF00"), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(arc_rect, 90 * 16, 180 * 16)

        # === Marker position ===
        # Clamp roll value
        clamped_roll = max(min(self.roll, 90.0), -90.0)

        # Map roll: +90 (top) → 0 (middle) → -90 (bottom)
        # Angle in radians: 180° (top), 270° (middle), 360° (bottom)
        angle_deg = 180 + clamped_roll  # So +90 becomes 270, 0 becomes 180, -90 becomes 90
        angle_rad = math.radians(-angle_deg)

        marker_x = arc_center.x() + arc_radius * math.cos(angle_rad)
        marker_y = arc_center.y() - arc_radius * math.sin(angle_rad)
        marker_pos = QPointF(marker_x, marker_y)

        # Draw marker
        painter.setBrush(QBrush(QColor("#FF4500")))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(marker_pos, 5, 5)

        # Draw roll value
        painter.setPen(QColor("#FF4500"))
        painter.setFont(QFont("Courier New", 12, QFont.Bold))
        painter.drawText(marker_pos + QPointF(10, 5), f"{self.roll:.1f}°")

"""         # === Vertical Pitch Scale (Right Side) ===
        bar_x = float(width - 30)
        scale_factor = 0.75  # tweak this value to align triangle and scale
        scale_height = equilateral_height * scale_factor
        bar_top = center.y() - scale_height
        bar_bottom = center.y() + scale_height

        tick_spacing_deg = 10
        tick_length = 8
        bar_color = QColor("#00FFFF")  # Match triangle blue
        text_color = QColor("#FF4500")  # Red-orange for pitch value

        # Compute marker position (match triangle nose)
        clamped_pitch = max(min(self.pitch, self.max_angle), -self.max_angle)
        normalized_pitch = clamped_pitch / self.max_angle
        marker_y = center.y() + nose_y  # nose_y already includes proper pitch direction and scale


        # === 1. Draw pitch value (left of bar, red)
       # painter.setPen(QPen(text_color))
       # painter.setFont(QFont("Courier New", 10))
       # painter.drawText(QPointF(bar_x - 60, marker_y + 5), f"{self.pitch:+.1f}°")

        # === 2. Draw pitch bar line
        painter.setPen(QPen(bar_color, 2))
        painter.drawLine(QPointF(bar_x, bar_top), QPointF(bar_x, bar_bottom))

        # === 3. Draw red circle marker (on top of bar)
    #    painter.setPen(Qt.NoPen)
    #    painter.setBrush(QBrush(text_color))
    #    painter.drawEllipse(QPointF(bar_x, marker_y), 5.0, 5.0)

        # === 4. Draw ticks and graduation labels
        painter.setPen(QPen(bar_color, 1))
        painter.setFont(QFont("Courier New", 8))
        for deg in range(-90, 91, tick_spacing_deg):
            norm = deg / self.max_angle
            y = center.y() - norm * scale_height


            # Tick line
            painter.drawLine(QPointF(bar_x - tick_length, y), QPointF(bar_x + tick_length, y))

            # Label only every 30°
            if deg % 30 == 0:
                painter.drawText(QPointF(bar_x + 10, y + 4), f"{deg:+d}°") """













