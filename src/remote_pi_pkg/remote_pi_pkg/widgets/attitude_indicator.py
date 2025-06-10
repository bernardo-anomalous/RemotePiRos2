
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QTransform, QFont
from PyQt5.QtCore import Qt, QPointF, QRectF
import math

class AttitudeIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0

        self.setMinimumHeight(250)
        self.setMinimumWidth(200)

        # === Customizable Style Parameters ===
        self.base_width = 140
        # Triangle base half-width
        self.pitch_effect_scale = 1.0
        self.max_angle = 90.0  # Maximum pitch/roll for normalization

        self.outer_color = QColor("#00FFFF")    # Aqua (pose triangle)
        self.intensity_color = QColor("#FF4500")  # Orange-red (intensity fill)
        self.horizon_color = QColor("#00FF00")  # Green (horizon lines)
        self.target_heading = 0.0  # Add this next to roll and pitch
        self.heading = 0.0                # Current smoothed heading
        self.target_heading = 0.0         # Target heading from update
        self.current_depth = 0.0
        self.target_depth = 0.0   # Updated directly from the ROS topic
        self.target_accel_x = 0.0
        self.target_accel_y = 0.0
        self.display_accel_x = 0.0
        self.display_accel_y = 0.0
        self.visual_gain = 100.0  # Amplify acceleration visually


    def set_acceleration(self, accel_x, accel_y, accel_z):
        """Called by ROSInterface to update acceleration vector."""
        # print statements removed to avoid console spam

        try:
            self.target_accel_x = float(accel_y) * self.visual_gain      # Swap X and Y
            self.target_accel_y = float(accel_x) * self.visual_gain     # Swap X and Y + invert

            self.update()  # Important: triggers repaint
        except Exception as e:
            # log the error if needed; suppress direct printing
            pass





    def update_attitude_target(self, euler):
        """Set the target roll and pitch (called by the GUI)."""
        if euler and len(euler) >= 2:
            try:
                self.target_roll = float(euler[0])
                self.target_pitch = float(euler[1])
            except Exception:
                pass
            
    def smooth_update_value(self, current, target, factor=0.2):
        return current + factor * (target - current)
    
    def update_heading_target(self, heading):
        self.target_heading = heading % 360
        
    def draw_heading_reference_grid(self, painter):
        rect = self.rect()
        width = rect.width()
        height = rect.height()
        center_x = width // 2

        spacing = 40                        # Spacing between grid lines
        pixels_per_degree = 5.0             # How much drift per degree

        grid_width_for_full_rotation = 360 * pixels_per_degree
        shift_x = - (self.heading % 360) * pixels_per_degree

        # Now calculate enough lines to cover the total grid range plus the screen
        num_lines = int((grid_width_for_full_rotation // spacing) + (width // spacing)) + 4

        line_color_base = QColor(150, 150, 150)  # Faint gray
        max_opacity = 80

        for i in range(-num_lines, num_lines + 1):
            line_x = center_x + i * spacing + shift_x

            # Fade out near the edges
            distance_from_center = abs(line_x - center_x)
            fade = max(0.0, 1.0 - (distance_from_center / (width / 2)))
            alpha = int(max_opacity * fade)

            color = QColor(line_color_base)
            color.setAlpha(alpha)

            painter.setPen(QPen(color, 1))
            painter.drawLine(int(line_x), 0, int(line_x), height)
            
    def draw_pitch_reference_grid(self, painter):
        rect = self.rect()
        width = rect.width()
        height = rect.height()
        center_y = rect.center().y()

        pitch_spacing_deg = 10           # <<< Degrees between pitch lines (adjustable)
        max_pitch = int(self.max_angle)
       # Uses your existing max_angle (90°)
        pixels_per_degree = height / (2 * max_pitch)  # Scale pitch range to widget height

        line_color_base = QColor(255, 255, 0)  # Faint yellow
        max_opacity = 80

        # Draw lines from -max_pitch to +max_pitch
        pitch_offset = self.pitch  # Smoothed current pitch value

        for pitch_deg in range(-max_pitch, max_pitch + 1, pitch_spacing_deg):
            y_offset = -(pitch_deg - pitch_offset) * pixels_per_degree  # Shift lines based on pitch!
            line_y = center_y + y_offset

            if line_y < 0 or line_y > height:
                continue  # Skip lines outside the visible area

            # Fade out near top/bottom edges
            distance_from_center = abs(line_y - center_y)
            fade = max(0.0, 1.0 - (distance_from_center / (height / 2)))
            alpha = int(max_opacity * fade)

            color = QColor(line_color_base)
            color.setAlpha(alpha)

            # Thicker or highlighted line at 0° (level)
            if pitch_deg == 0:
                painter.setPen(QPen(color, 3))
            else:
                painter.setPen(QPen(color, 1))

            painter.drawLine(0, int(line_y), width, int(line_y))








    def paintEvent(self, event):
        # === Smooth updates ===
        self.roll = self.smooth_update_value(self.roll, self.target_roll)
        self.pitch = self.smooth_update_value(self.pitch, self.target_pitch)
        delta = (self.target_heading - self.heading + 540) % 360 - 180
        self.heading = (self.heading + 0.2 * delta) % 360

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), Qt.transparent)

        rect = self.rect()
        center = rect.center()
        center_x = center.x()
        center_y = center.y()
        width = rect.width()
        height = rect.height()

        # === Reference Grids ===
        self.draw_pitch_reference_grid(painter)
        self.draw_heading_reference_grid(painter)

        # === Normalized Pitch/Roll ===
        norm_pitch = max(min(self.pitch / self.max_angle, 1.0), -1.0)
        norm_roll = max(min(self.roll / self.max_angle, 1.0), -1.0)
        pitch_intensity = abs(norm_pitch)

        # === Triangle Geometry ===
        base_half_width = self.base_width
        equilateral_height = base_half_width * (3**0.5)
        nose_y = -equilateral_height * norm_pitch

        point_nose = QPointF(0, nose_y)
        point_left = QPointF(-base_half_width, 0)
        point_right = QPointF(base_half_width, 0)
        triangle = [point_nose, point_left, point_right]

        transform = QTransform()
        transform.translate(center_x, center_y)
        transform.rotate(self.roll)
        transformed = [transform.map(p) for p in triangle]

        # === Outer Triangle ===
        painter.setBrush(QBrush(self.outer_color))
        painter.setPen(QPen(self.outer_color, 2))
        painter.drawPolygon(*transformed)

        # === Nose Line to Pitch Scale ===
        nose_tip = transform.map(point_nose)
        line_end_x = width - 140
        line_end = QPointF(line_end_x, nose_tip.y())

        painter.setPen(QPen(QColor("#00FF00"), 1, Qt.DashLine))
        painter.drawLine(nose_tip, line_end)

        painter.setPen(QPen(QColor("#FF4500")))
        painter.setFont(QFont("Courier New", 12, QFont.Bold))
        painter.drawText(line_end + QPointF(5, 5), f"{self.pitch:+.1f}°")

        # === New Triangle Scaling ===
        # Scale from 0.5 (smallest) to 1.5 (largest) depending on pitch
        inner_scale = 0.1 + (abs(self.pitch) / self.max_angle)  # 1.0 at 0°, 2.0 at 90°

        # === New Color Behavior ===
        if abs(self.pitch) > 25.0:
            fill_color = QColor("#FF0000")  # Bright RED if pitch out of safe range
        else:
            fill_color = QColor("#00AA00")  # Strong GREEN if safe

        inner_triangle = [QPointF(p.x() * inner_scale, p.y() * inner_scale) for p in triangle]
        inner_transformed = [transform.map(p) for p in inner_triangle]

        painter.setBrush(QBrush(fill_color))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(*inner_transformed)

        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor("#00FF00"), 1))
        painter.drawPolygon(*inner_transformed)

        # === Horizon Markers ===
        painter.setPen(QPen(self.horizon_color, 2))
        painter.drawLine(10, center_y, 30, center_y)
        painter.drawLine(width - 30, center_y, width - 10, center_y)

        # === Roll Arc & Marker ===
        arc_radius = 140
        arc_center = QPointF(260, center_y)
        arc_rect = QRectF(arc_center.x() - arc_radius,
                        arc_center.y() - arc_radius,
                        arc_radius * 2,
                        arc_radius * 2)

        painter.setPen(QPen(QColor("#00FF00"), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(arc_rect, 130 * 16, 100 * 16)

        clamped_roll = max(min(self.roll, 90.0), -90.0)
        angle_deg = 180 + clamped_roll
        angle_rad = math.radians(-angle_deg)

        marker_x = arc_center.x() + arc_radius * math.cos(angle_rad)
        marker_y = arc_center.y() - arc_radius * math.sin(angle_rad)
        marker_pos = QPointF(marker_x, marker_y)

        painter.setBrush(QBrush(QColor("#FF4500")))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(marker_pos, 5, 5)

        painter.setPen(QColor("#FF4500"))
        painter.setFont(QFont("Courier New", 12, QFont.Bold))
        painter.drawText(marker_pos + QPointF(-70, 5), f"{self.roll:.1f}°")

        # === Depth Gauge ===
        if hasattr(self, 'current_depth') and hasattr(self, 'target_depth'):
            deadband = 0.01
            target = self.target_depth

            if abs(target) < deadband:
                target = 0.0

            if target != 0.0:
                smoothing_factor = 0.1
                self.current_depth += (target - self.current_depth) * smoothing_factor
            else:
                self.current_depth = 0.0

            font = QFont("Courier", 14)
            painter.setFont(font)
            painter.setPen(QColor("#00FFFF"))

            current = round(self.current_depth, 2)
            previous = round(current - 1, 2)
            next_val = round(current + 1, 2)

            x_offset = self.width() - 80
            y_center_text = 70

            painter.setOpacity(0.3)
            painter.drawText(x_offset, y_center_text - 20, f"{previous}m")

            painter.setOpacity(1.0)
            painter.drawText(x_offset, y_center_text, f"{current}m")

            painter.setOpacity(0.3)
            painter.drawText(x_offset, y_center_text + 20, f"{next_val}m")

            painter.setOpacity(1.0)

        # === Moving Acceleration Dot + Ghost Trail ===
        fast_factor = 0.4
        slow_decay = 0.98

        self.display_accel_x += (self.target_accel_x - self.display_accel_x) * fast_factor
        self.display_accel_y += (self.target_accel_y - self.display_accel_y) * fast_factor

        if abs(self.target_accel_x) < 0.05 and abs(self.target_accel_y) < 0.05:
            self.display_accel_x *= slow_decay
            self.display_accel_y *= slow_decay

        if not hasattr(self, 'trail_points'):
            self.trail_points = []
            self.max_trail_length = 15

        current_dot_pos = (center_x + self.display_accel_x, center_y + self.display_accel_y)
        self.trail_points.append(current_dot_pos)

        if len(self.trail_points) > self.max_trail_length:
            self.trail_points.pop(0)

        # === Draw ghost trail ===
        for i, pos in enumerate(self.trail_points):
            alpha = int(255 * (i + 1) / self.max_trail_length)
            ghost_color = QColor(255, 255, 255, alpha)
            painter.setBrush(QBrush(ghost_color))
            painter.setPen(Qt.NoPen)
            dot_size = max(2, 6 - int(i / 2))
            painter.drawEllipse(
                int(pos[0]) - dot_size // 2,
                int(pos[1]) - dot_size // 2,
                dot_size,
                dot_size
            )

        # === Draw final white dot ===
        dot_radius = 4
        painter.setBrush(QBrush(Qt.white))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(
            int(current_dot_pos[0]) - dot_radius,
            int(current_dot_pos[1]) - dot_radius,
            dot_radius * 2,
            dot_radius * 2
        )

        
    def set_depth(self, depth_value):
        self.target_depth = depth_value  # Target value from the topic
        #print(f"Depth updated: {depth_value}")
